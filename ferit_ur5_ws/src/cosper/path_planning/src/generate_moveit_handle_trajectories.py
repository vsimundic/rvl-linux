#!/usr/bin/python

import rospy
import os
from core.util import read_config, read_csv_DataFrame
from core.ur5_commander import UR5Commander
from gazebo_push_open.cabinet_model import Cabinet
import numpy as np
from core.transforms import rot_z, pose_to_matrix
from gazebo_msgs.msg import ContactsState
from core.gazebo import get_link_pose
import csv
from gazebo_msgs.msg import ContactsState
from rospkg import RosPack
from tf2_ros import Buffer, TransformListener
from utils import *
from tqdm import tqdm
import RVLPYDDManipulator as rvlpy_dd_man

def rvl_generate_handle_path(rvl_ik_solver, robot: UR5Commander, cabinet_model:Cabinet, opening_angles, T_T_B_handle_rots, q_init):
    q_dist_thresh = 0.25* np.pi
    # Generate opening traj with handle until its not possible anymore
    q_moveit_traj = [q_init]

    np.save('/home/RVLuser/ferit_ur5_ws/T_T_B_handle.npy', T_T_B_handle_rots[0])

    # Generate multiple first points from which it will expand
    # q, n_sol, _ = rvl_ik_solver.inv_kinematics_all_sols(T_T_B_handle_rots[0], bTCP)
    q, n_sol, _ = rvl_ik_solver.inv_kinematics_all_sols_prev(T_T_B_handle_rots[0])
    q[:, 0] -= np.pi
    # q[:, 5] -= np.pi
    q[q>np.pi]-=(2.0*np.pi)     
    q[q<-np.pi]+=(2.0*np.pi)

    # Set cabinet model state
    cabinet_model.change_door_angle(opening_angles[0])
    cabinet_model.update_mesh()
    cabinet_model.save_mesh(cabinet_full_mesh_filename)
    robot.remove_from_scene('cabinet')
    robot.add_mesh_to_scene(cabinet_full_mesh_filename, 'cabinet', T_A_S)

    if n_sol < 1:
        return np.array(q_moveit_traj)
    mask = np.array([robot.check_joint_feasibility(q_moveit_traj[0], q[i_sol].tolist()) for i_sol in range(n_sol)])
    q_sols = q[:n_sol][mask]
    if q_sols.shape[0] < 1:
        return np.array(q_moveit_traj)
    
    # q_sols = [q_sols.tolist()] # now we get first joints
    q_sols = [[q_sol] for q_sol in q_sols.tolist()] # now we get first joints
    
    longest_traj = []

    for i_q in range(len(q_sols)):
        q_arr = q_sols[i_q]
        q_last = q_arr[-1]

        for i_state in range(1, T_T_B_handle_rots.shape[0]):
            T_T_B_ = T_T_B_handle_rots[i_state]

            # q_, n_sol, _ = rvl_ik_solver.inv_kinematics_all_sols(T_T_B_, False)
            q_, n_sol, _ = rvl_ik_solver.inv_kinematics_all_sols_prev(T_T_B_)
            q_[:, 0] -= np.pi
            # q_[:, 5] -= np.pi
            q_[q_>np.pi]-=(2.0*np.pi)     
            q_[q_<-np.pi]+=(2.0*np.pi)

            if n_sol < 1:
                break
                
            # Set cabinet model state
            cabinet_model.change_door_angle(opening_angles[i_state])
            cabinet_model.update_mesh()
            cabinet_model.save_mesh(cabinet_full_mesh_filename)
            robot.remove_from_scene('cabinet')
            robot.add_mesh_to_scene(cabinet_full_mesh_filename, 'cabinet', T_A_S)

            # Check if there is a feasible solution
            min_dist = np.inf
            idx_min = -1
            for i_sol in range(n_sol):
                q__ = q_[i_sol].tolist()

                dist_ = chebyshev_distance(q_last, q__)
                if dist_ > q_dist_thresh:
                    continue

                if not robot.check_joint_feasibility(q_last, q__):
                    continue

                if dist_ < min_dist:
                    min_dist = dist_
                    idx_min = i_sol

            if idx_min > -1:
                q_arr.append(q_[idx_min].tolist())
            else:
                break
        
        if len(q_arr) > len(longest_traj):
            longest_traj = q_arr
    
    if longest_traj:
        longest_traj.insert(0, q_init.tolist())

    return np.array(longest_traj) if longest_traj else np.array(q_moveit_traj)


    # for i_state in range(T_T_B_handle_rots.shape[0]):
    #     T_T_B_ = T_T_B_handle_rots[i_state]
        
    #     q_state = None
    #     for _ in range(10): # 5 tries to get inverse kin that doesn't have wild joint rotations 
    #         q, n_sol, success = rvl_ik_solver.inv_kinematics_all_sols(T_T_B_, False)
    #         if not success:
    #             continue
    #         for i_sol in range(n_sol):
    #             pass
                
    #         q_state = q[0]
    #         # q_state = robot.get_inverse_kin(q_moveit_traj[-1], T_T_B_)
    #         if q_state is None:
    #             continue # possibly could just go with break
    #         q_state = np.array(q_state)
    #         q_state[q_state>np.pi]-=(2.0*np.pi)     
    #         q_state[q_state<-np.pi]+=(2.0*np.pi)

    #         if i_state == 1:
    #             break
    #         else:
    #             if chebyshev_distance(q_moveit_traj[-1], q_state) < qdist_thresh: # check cheb dist for wild joint rotations
    #                 break
    #             else:
    #                 q_state = None
    #     if q_state is None:
    #         break
    #     q_moveit_traj.append(q_state.tolist())
    
    # q_moveit_traj = np.array(q_moveit_traj)
    # # q_moveit_traj[q_moveit_traj>np.pi]-=(2.0*np.pi)     
    # # q_moveit_traj[q_moveit_traj<-np.pi]+=(2.0*np.pi)
    # q_moveit_traj[1:] = np.unwrap(q_moveit_traj[1:], axis=0, discont=np.pi*0.9)

    # return q_moveit_traj

if __name__ == '__main__':
    rospy.init_node('generate_moveit_handle_trajectories')

    IS_SAVING_RESULTS = True
    IS_SAVING_IMAGES = False
    START_FROM_BEGINNING = True
    method_name = 'our'

    # TF buffer setup
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    rp = RosPack()
    pkg_path = rp.get_path('path_planning')
    
    # Config
    cfg_path = os.path.join(pkg_path, 'config/config_multi-c_%s_handle_axis_left.yaml' % method_name)
    config = read_config(cfg_path)
    
    # Moveit trajectories
    trajectories_path = config['moveit_trajectories_path']
    if not os.path.exists(trajectories_path):
        os.makedirs(trajectories_path)

    # Gazebo simulation launch file (UR5 moveit config launch)
    gazebo_launch_file = config['gazebo_launch_path']

    # Load door configurations
    door_configs_path = config['cabinet_configs_path']
    doors = np.load(door_configs_path)
    num_doors = doors.shape[0]
    
    T_G_T = np.load(config['gripper_tool_pose'])
    T_R_W = np.load(config['robot_world_pose'])
    T_W_R = np.linalg.inv(T_R_W)

    # Tool pose w.r.t. handle (it will always be the same)
    axis_pos = -1.0
    Tz45 = np.eye(4)
    Tz45[:3,:3] = rot_z(np.radians(45.))
    T_T_H = np.eye(4)
    T_T_H[:3, :3] = np.array([[0, 0, -axis_pos],
                                [axis_pos, 0, 0],
                                [0, -1, 0]])
    T_T_H = T_T_H @ Tz45
    T_T_H[0, 3] -= 0.28

    # Initial pose in joint space
    q_init = np.array([0., -np.pi*0.5, 0., -np.pi*0.5, 0., 0.])
    q_init_ros = q_init.copy()
    # Adjust joint values from ROS
    q_init[0] += np.pi
    q_init[5] += np.pi

    # Static cabinet params
    door_thickness=config['cabinet_door_dims']['depth']
    static_depth=config['cabinet_door_dims']['static_depth']

    # Static mesh filename
    cabinet_static_mesh_filename = config['cabinet_static_mesh_save_path']
    cabinet_panel_mesh_filename = config['cabinet_panel_mesh_save_path']
    cabinet_full_mesh_filename = config['cabinet_full_mesh_save_path']

    # IK RVL setup
    rvl_cfg_path = config['rvl_config_path']
    rvl_ik_solver = rvlpy_dd_man.PYDDManipulator()
    rvl_ik_solver.create(rvl_cfg_path)
    rvl_ik_solver.set_robot_pose(T_R_W)

    # Generate rotations for all states
    num_states = 30
    opening_angles = np.linspace(0, axis_pos*np.pi*0.5, num=num_states)
    cos_vals = np.cos(opening_angles)
    sin_vals = np.sin(opening_angles)
    Tz_rots = np.zeros((num_states,4,4))
    Tz_rots[:,0,0] = cos_vals
    Tz_rots[:,0,1] = -sin_vals
    Tz_rots[:,1,0] = sin_vals
    Tz_rots[:,1,1] = cos_vals
    Tz_rots[:,2,2] = 1.
    Tz_rots[:,3,3] = 1.

    # Chebyshev joint distance threshold
    qdist_thresh = 0.25 * np.pi

    # Start Gazebo processes
    kill_ros_nodes()
    rospy.sleep(1.)
    kill_processes()
    reset_tf_buffer(tf_buffer)
    launch_process = subprocess.Popen("source /home/RVLuser/ferit_ur5_ws/devel/setup.bash && roslaunch ur5_robotiq_ft_3f_moveit_config demo_gazebo.launch", shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    rospy.sleep(4.)

    # Start the robot commander
    try:
        robot = UR5Commander()
    except RuntimeError as e:
        kill_processes()
        launch_process.terminate()
        launch_process.wait()
        exit()

    # Sleep to ensure the robot is ready
    rospy.sleep(1.)

    # Tqdm
    pbar = tqdm(total = num_doors)

    start_i = 0
    i = start_i
    while i < num_doors:
        # print('Cabinet %d' % i)
        try:            
            # Cabinet parameters
            door = doors[i, :]
            width = door[0]
            height = door[1]
            position = door[2:5]
            rot_z_deg = door[5]
            state_angle = door[6]
            axis_pos = door[7]

            T_A_S = np.eye(4)
            T_A_S[:3, 3] = np.array(position)
            Tz = np.eye(4)
            Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
            T_A_S = T_A_S @ Tz

            # Create a cabinet object
            cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                                    axis_pos=axis_pos,
                                    T_A_S=T_A_S,
                                    save_path=config['cabinet_urdf_save_path'],
                                    has_handle=True)
            
            # Save cabinet mesh to a file
            cabinet_model.save_mesh_without_doors(cabinet_static_mesh_filename)
            cabinet_model.save_door_panel_mesh(cabinet_panel_mesh_filename)
            # cabinet_model.change_door_angle(state_angle)
            # cabinet_model.update_mesh()
            # Save full mesh
            cabinet_model.save_mesh(cabinet_full_mesh_filename)

            # First, go to the door handle pose
            T_T_B_handle = T_W_R @ T_A_S @ cabinet_model.T_H_A @ T_T_H

            # Calc robot tool poses for each opening state
            T_A_S_rots = T_A_S[np.newaxis,...] @ Tz_rots
            T_T_B_handle_rots = T_W_R[np.newaxis,...] @ T_A_S_rots @ cabinet_model.T_H_A[np.newaxis,...] @ T_T_H[np.newaxis,...]
            T_G_0_handle_rots = T_W_R[np.newaxis,...] @ T_A_S_rots @ cabinet_model.T_H_A[np.newaxis,...] @ T_T_H[np.newaxis,...] @ T_G_T[np.newaxis,...]
            
            # # Spawning model in Gazebo
            # cabinet_model.delete_model_gazebo()
            # cabinet_model.spawn_model_gazebo()

            robot.remove_from_scene('cabinet')
            robot.add_mesh_to_scene(cabinet_full_mesh_filename, 'cabinet', T_A_S)


            # q, n_sol, _ = rvl_ik_solver.inv_kinematics_all_sols(T_T_B_handle_rots[0], False)

            q_moveit_traj = rvl_generate_handle_path(rvl_ik_solver, robot, cabinet_model, opening_angles, T_T_B_handle_rots, q_init)


            if False:

                # Generate opening traj with handle until its not possible anymore
                q_moveit_traj = [q_init_ros.tolist()]
                # for i_state in range(1, T_T_B_handle_rots.shape[0] + 1):
                for i_state in range(1, T_G_0_handle_rots.shape[0] + 1):
                    T_T_B_ = T_T_B_handle_rots[i_state - 1]
                    T_G_0_ = T_G_0_handle_rots[i_state - 1]
                    
                    q_state = None
                    for _ in range(10): # 5 tries to get inverse kin that doesn't have wild joint rotations 
                        # q_state, _ = robot.get_inverse_kin_builtin(q_moveit_traj[-1], T_T_B_)
                        # q_state = robot.get_inverse_kin(q_moveit_traj[-1], T_G_0_)
                        q_state = robot.get_inverse_kin(q_moveit_traj[-1], T_T_B_)
                        if q_state is None:
                            continue # possibly could just go with break
                        q_state = np.array(q_state)
                        q_state[q_state>np.pi]-=(2.0*np.pi)     
                        q_state[q_state<-np.pi]+=(2.0*np.pi)

                        if i_state == 1:
                            break
                        else:
                            if chebyshev_distance(q_moveit_traj[-1], q_state) < qdist_thresh: # check cheb dist for wild joint rotations
                                break
                            else:
                                q_state = None
                    if q_state is None:
                        break
                    q_moveit_traj.append(q_state.tolist())
                
                q_moveit_traj = np.array(q_moveit_traj)
                # q_moveit_traj[q_moveit_traj>np.pi]-=(2.0*np.pi)     
                # q_moveit_traj[q_moveit_traj<-np.pi]+=(2.0*np.pi)
                q_moveit_traj[1:] = np.unwrap(q_moveit_traj[1:], axis=0, discont=np.pi*0.9)


            np.savetxt(os.path.join(trajectories_path, 'traj_%d.csv' % i), q_moveit_traj, delimiter=",", fmt="%.6f")

            i+=1
            pbar.update(1)
        
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(f"Time moved backwards. Exception: {e}")
            rospy.loginfo(f"Current ROS time: {rospy.Time.now()}")
            rospy.sleep(2)  # Let time stabilize


