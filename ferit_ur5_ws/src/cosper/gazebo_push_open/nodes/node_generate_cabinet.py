#!/usr/bin/python

import rospy
import os
import rospkg
from core.read_config import read_config
from core.paths_packages import get_package_name_from_node, get_package_path_from_name
from core.ur5_commander import UR5Commander
from DDMan import push
from gazebo_push_open.cabinet_model import Cabinet
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
from core.transforms import rot_z, matrix_to_pose


if __name__ == '__main__':
    rospy.init_node('node_generate_cabinet')
    
    try:
        cfg_file = rospy.get_param('config_file')
    except rospy.exceptions.ROSException:
        raise Exception('Could not fetch param.')
    
    config = read_config(cfg_file)
    
    
    #### CABINET ####
    cabinet_door_dims = config['cabinet_door_dims'] # w_door, h_door, static_d, d_door 
    door_params = [np.random.uniform(cabinet_door_dims['min_width'], cabinet_door_dims['max_width']), 
                  np.random.uniform(cabinet_door_dims['min_height'], cabinet_door_dims['max_height']),
                  cabinet_door_dims['depth'],
                  cabinet_door_dims['static_depth']]
    
    cabinet_pose = config['cabinet_pose']
    feasible_poses_args = config['feasible_poses']

    # Cabinet pose in world
    cabinet_position = [np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x']),
                        np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y']),
                        door_params[1]/2 + 0.018 + 0.01 + 0.02] # half of door height + depth of bottom panel + double static moving part distance
    rotz_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])

    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array(cabinet_position)
    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.radians(rotz_deg))
    T_A_S = T_A_S @ Tz


    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array(door_params), 
                            axis_pos=cabinet_pose['axis_pos'],
                            T_A_S=T_A_S,
                            # position=np.array(cabinet_position),
                            # rotz_deg=rotz_deg,
                            save_path=config['cabinet_urdf_save_path'])
    
    # Save cabinet mesh to a file
    cabinet_model.save_mesh_without_doors(config['cabinet_mesh_save_path'])

    # Spawning model in Gazebo
    cabinet_model.delete_model_gazebo()
    cabinet_model.spawn_model_gazebo()
    T_D_S_gazebo = cabinet_model.get_door_panel_RF_wrt_world()

    # Open doors in Gazebo
    theta_deg = config['feasible_poses']['dd_state_deg']
    cabinet_model.open_door_gazebo(theta_deg)
    cabinet_model.change_door_angle(theta_deg)
    cabinet_model.update_mesh()



    #### FEASIBLE POSES ####
    # Get feasible poses
    feasible_poses_args['door_dims'] = np.array(door_params[:3])
    feasible_poses_args['static_depth'] = door_params[3]
    feasible_poses = push.demo_push_poses_ros(**feasible_poses_args)
    
    # Pick one pose
    T_G_D = feasible_poses[2]
        
    # Get robot gripper pose in world
    T_G_S = cabinet_model.get_feasible_pose_wrt_world(T_G_D)
    
    # Pose further away for approach
    T_D_G_prev = np.linalg.inv(T_G_D)
    T_D_G_prev[2, 3] += 0.30
    T_G_D_prev = np.linalg.inv(T_D_G_prev)
    T_G_S_prev = cabinet_model.get_feasible_pose_wrt_world(T_G_D_prev)

    # # Open3D visualization
    cabinet_model.visualize(T_G_S_prev)
    
    # Helping matrices
    # T_B_S = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_B_0.npy')))
    # T_G_T = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_G_T.npy')))
    
    # T_G_T = np.eye(4)
    # T_G_T[:3, :3] = rot_z(np.radians(-90))
    # T_G_T[2, 3] = 0.115
    # print(T_G_T)
    # np.save(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TGT.npy'), T_G_T)
    
    # Robot handler
    robot = UR5Commander()

    # Go to home pose
    q_init = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/q_init.npy')
    # q_init = robot.get_current_joint_values()
    # np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/q_init.npy', np.array(q_init))
    robot.send_joint_values_to_robot(q_init.tolist())

    # # Add cabinet mesh to rviz
    # robot.add_mesh_to_scene(config['cabinet_mesh_save_path'], 'cabinet', np.linalg.inv(robot.T_B_S) @ cabinet_model.T_O_S)

    # Calculate poses
    T_T_B_fp_prev = robot.get_tool_pose_from_gripper_pose(T_G_S_prev) # approaching pose
    T_T_B_fp = robot.get_tool_pose_from_gripper_pose(T_G_S) # feasible pose
    T_G_S_arr = cabinet_model.generate_opening_passing_poses(T_G_D, max_angle_deg=70., num_poses=10) # passing poses
    T_T_B_arr = [robot.get_tool_pose_from_gripper_pose(T_G_S_) for T_G_S_ in T_G_S_arr]

    q_temp = list(q_init.copy())
    q = []
    for i in range(len(T_T_B_arr)):
        q_ = robot.get_inverse_kin(q_temp, T_T_B_arr[i])
        if q_ is not None:
            q.append(q_)
        q_temp = q[len(q)-1]

    # # Add door panel to planning scene to stop collision when approaching
    # panel_rviz_name = 'door_panel'
    # T_D_B = np.linalg.inv(robot.T_B_S) @ cabinet_model.T_O_S @ cabinet_model.T_A_O @ cabinet_model.T_D_A_init
    # panel_size = (cabinet_model.d_door, cabinet_model.w_door, cabinet_model.h_door)
    # robot.add_box_to_scene(object_name=panel_rviz_name, pose=T_D_B, frame_id='base_link', size=panel_size)

    # Approaching points - feasible pose
    robot.send_pose_to_robot(T_T_B_fp_prev)
    # rospy.sleep(2)
    robot.send_pose_to_robot(T_T_B_fp, cartesian=True)
    rospy.sleep(0.5)

    # # Remove panel from rviz because of the collision
    # robot.remove_from_scene(name=panel_rviz_name)

    robot.send_multiple_joint_space_poses_to_robot(q, wait=True)

    # Passing poses 
    # robot.send_multiple_poses_to_robot(T_T_B_arr, wait=True, cartesian=True)

