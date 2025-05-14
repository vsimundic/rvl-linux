#!/usr/bin/python

import rospy
import os
import numpy as np
import json
import csv
import subprocess
from rospkg import RosPack
from tf2_ros import Buffer, TransformListener
from gazebo_msgs.msg import ContactsState
import sys
from core.util import read_config
from core.ur5_commander import UR5Commander
from core.transforms import rot_z
from gazebo_push_open.cabinet_model import Cabinet
from core.gazebo import get_link_pose
from force_utils import chebyshev_distance
from push_force_trajectories import generate_tool_line_poses, generate_trajectories, generate_trajectories_and_approach
sys.path.append(os.path.join(os.path.dirname(__file__), '../../path_planning/src'))
from utils import contact_callback, kill_processes, reset_tf_buffer
import RVLPYDDManipulator as rvlpy

def rotz_multiple(theta_arr):
    num_angles = theta_arr.shape[0]
    Rz = np.zeros((num_angles, 3, 3))
    c = np.cos(theta_arr)
    s = np.sin(theta_arr)
    Rz[:, 0, 0] = c
    Rz[:, 0, 1] = -s
    Rz[:, 1, 0] = s
    Rz[:, 1, 1] = c
    Rz[:, 2, 2] = 1
    return Rz

def generate_T_A_W_grid(
    x_range=(-0.36, 0.0), y_range=(0.41, 0.71),
    z_value=0.4, rot_range_deg=(-20, 20), step=0.02
):
    x_vals = np.arange(x_range[0], x_range[1] + step, step)
    y_vals = np.arange(y_range[0], y_range[1] + step, step)
    theta_deg = np.arange(rot_range_deg[0], rot_range_deg[1] + 5, 5)
    theta_rad = np.radians(theta_deg)

    X, Y, Theta = np.meshgrid(x_vals, y_vals, theta_rad, indexing='ij')
    num_transforms = X.size

    X_flat = X.ravel()
    Y_flat = Y.ravel()
    Theta_flat = Theta.ravel()

    cos_t = np.cos(Theta_flat)
    sin_t = np.sin(Theta_flat)
    R_ = np.array([[0, -1, 0],
                   [1, 0, 0],
                   [0, 0, 1]])
    R__ = np.tile(R_, (num_transforms, 1, 1))
    Rz = rotz_multiple(Theta_flat)

    R = R__ @ Rz

    T = np.zeros((num_transforms, 4, 4))
    T[:, :3, :3] = R
    T[:, 0, 3] = X_flat
    T[:, 1, 3] = Y_flat
    T[:, 2, 3] = z_value
    T[:, 3, 3] = 1.0

    return T


if __name__ == '__main__':
    rospy.init_node('single_cabinet_custom_trajectory_node')

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    # Static cabinet mesh filenames
    cabinet_static_mesh_filename = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_static_mesh.ply'
    cabinet_panel_mesh_filename = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_panel_mesh.ply'
    cabinet_full_mesh_filename = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_full_mesh.ply'
    cabinet_urdf_save_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_model.urdf'
    trajs_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/trajectories.npy'

    # Load door model info
    door_model_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/models/doorModel.json'
    with open(door_model_path, 'r') as f:
        data = json.load(f)

    R = np.array(data["R"])
    t = np.array(data["t"])
    s = np.array(data["s"])
    r = np.array(data["r"])
    axis_pos = data["openingDirection"]
    T_A_C = np.eye(4)
    T_A_C[:3, :3] = R
    T_A_C[:3, 3] = t

    T_0_W = np.eye(4)
    T_0_W[2, 3] = 0.005 # z offset of the robot from ground
    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')
    rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

    q_init = np.array([0., -np.pi*0.5, 0., -np.pi*0.5, 0., 0.])
    q_init[0] += np.pi
    q_init[5] += np.pi
    q_init[q_init > np.pi] -= (2.0 * np.pi)
    q_init[q_init < -np.pi] += (2.0 * np.pi)

    door_thickness = 0.018
    static_depth = 0.4

    # RVL manipulator to get forward kinematics
    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_cfg)
    rvl_manipulator.set_robot_pose(T_0_W)
    
    T_G_6 = rvl_manipulator.get_T_G_6()
    
    joint_values = np.array(data["joint_values"])
    joint_values[0] += np.pi
    joint_values[joint_values > np.pi] -= (2.0 * np.pi)
    joint_values[joint_values < -np.pi] += (2.0 * np.pi)
    # T_6_0 = rvl_manipulator.fwd_kinematics_6(joint_values)

    robot = UR5Commander()
    rospy.sleep(1.)

    T_6_0 = robot.forward_kinematics(joint_values.tolist())

    T_A_W_grid = generate_T_A_W_grid()

    # T_A_W = T_0_W @ T_6_0 @ T_C_6 @ T_A_C
    # T_A_W[2, 3] += 0.005 # z offset of the robot from ground
    # T_A_W[1, 3] -= 0.10
    # T_A_W[0, 3] -= 0.05
    width = s[0]
    height = s[1]
    push_latch_mechanism_length = 0.046 + door_thickness * 0.5
    state_angle = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / width))

    for i in range(T_A_W_grid.shape[0]):
        T_A_W = T_A_W_grid[i]
        cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]),
                                r=np.array(r),
                                axis_pos=axis_pos,
                                T_A_S=T_A_W,
                                save_path=cabinet_urdf_save_path,
                                has_handle=False)

        cabinet_model.save_mesh_without_doors(cabinet_static_mesh_filename)
        cabinet_model.save_door_panel_mesh(cabinet_panel_mesh_filename)
        cabinet_model.change_door_angle(state_angle)
        cabinet_model.update_mesh()
        cabinet_model.save_mesh(cabinet_full_mesh_filename)

        rvl_manipulator.set_door_model_params(
            cabinet_model.d_door,
            cabinet_model.w_door,
            cabinet_model.h_door,
            cabinet_model.rx,
            cabinet_model.ry,
            cabinet_model.axis_pos,
            cabinet_model.static_side_width,
            cabinet_model.moving_to_static_part_distance)
        rvl_manipulator.set_door_pose(cabinet_model.T_A_S)
        rvl_manipulator.set_environment_state(state_angle)

        # # Run Gazebo and robot
        # kill_processes()
        # reset_tf_buffer(tf_buffer)
        # launch_process = subprocess.Popen("source /home/RVLuser/ferit_ur5_ws/devel/setup.bash && roslaunch ur5_robotiq_ft_3f_moveit_config demo_gazebo.launch", shell=True, executable="/bin/bash")
        # rospy.sleep(4.)



        # Generate poses and trajectories
        R_TCP_D = np.array([[0, 0, -1], 
                            [0, 1, 0], 
                            [1, 0, 0]])
        T_TCP_G = np.eye(4)
        T_TCP_G[:3, 3] = np.array([0.155 / 2, 0, 0.098])
        T_G_DD_all = generate_tool_line_poses(height, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)

        # Debug trajectories with approach
        _, trajectories = generate_trajectories_and_approach(T_G_DD_all, 37, state_angle, -90.0, cabinet_model, rvl_cfg, T_0_W, robot)

        if len(trajectories) < 1:
            print("No trajectories generated.")
            # exit(0)
            continue

        print(f"Generated {len(trajectories)} trajectory roots.")

        # Execute each trajectory
        for i in range(trajectories.shape[0]):
            print(f"Executing trajectory {i}")

            trajectory = np.array(trajectories[i])
            # trajectory[:, 0] += np.pi
            # trajectory[trajectory < -np.pi] += 2 * np.pi
            # trajectory[trajectory > np.pi] -= 2 * np.pi
            if not robot.validate_trajectory_points(trajectory):
                continue
            
            # # Extract contact point
            # q_contact_rvl = trajectory[0, :6].copy() # copy because it is modified in the next line - for ROS
            # q_contact = trajectory[0, :6].copy()
            # q_contact[0] += np.pi
            # # q_contact[5] += np.pi

            # q_contact[q_contact > np.pi] -= (2.0 * np.pi)
            # q_contact[q_contact < -np.pi] += (2.0 * np.pi)
            # T_6_0_contact = robot.forward_kinematics(q_contact)
            # # rvl_manipulator.visualize_current_state(q_contact_rvl, T_6_0_contact @ T_G_6)
            # if T_6_0_contact is None:
            #     print("Invalid trajectory, skipping...")
            #     continue
            
            # T_G_W_contact = T_0_W @ T_6_0_contact @ T_G_6
            # T_G_0_contact = T_6_0_contact @ T_G_6

            # # Get approach path
            # T_G_0_via, ik_solutions, paths = rvl_manipulator.approach_path(T_G_W_contact.copy())
            # if T_G_0_via.shape[0] < 1 or len(paths) < 1: 
            #     print("No points in approach path...")
            #     continue

            # # Check if ik_solutions are valid in approach path
            # has_path = False
            # for i_path in range(len(paths)):
            #     path = paths[i_path]
            #     ik0 = np.array(ik_solutions[0][path[0]])
            #     ik1 = np.array(ik_solutions[1][path[1]])
            #     ik2 = ik_solutions[2][0]
            #     if chebyshev_distance(q_contact_rvl, ik1) > 0.5*np.pi:
            #         continue
            #     else:
            #         has_path = True
            #         trajectory = np.concatenate((ik0.reshape(1, -1), ik1.reshape(1, -1), trajectory), axis=0)
            #         break
            
            # if not has_path:
            #     print("No valid path found, skipping...")
            #     continue
            
            # # T_G_0_via, n_pts = rvl_manipulator.approach_path_poses(T_G_W_contact.copy())
            # rvl_manipulator.visualize_current_state(q_contact_rvl, T_6_0_contact @ T_G_6)

            trajectory[:, 0] -= np.pi
            trajectory[:, 5] -= np.pi
            trajectory[trajectory > np.pi] -= 2 * np.pi
            trajectory[trajectory < -np.pi] += 2 * np.pi
            trajectory = np.unwrap(trajectory, axis=0)

            # Delete the model if it exists
            cabinet_model.delete_model_gazebo()

            # Create the approach path
            approach_path = [robot.get_current_joint_values(), trajectory[0].tolist(), trajectory[1].tolist()]
            
            # Send the approach path to the robot
            robot.send_multiple_joint_space_poses_to_robot2(approach_path)
            
            # Spawn the cabinet model in Gazebo and set the door state
            cabinet_model.spawn_model_gazebo()
            cabinet_model.set_door_state_gazebo(state_angle)
            cabinet_model.change_door_angle(state_angle)

            # Send the opening trajectory to the robot
            robot.send_multiple_joint_space_poses_to_robot2(trajectory[1:].tolist())
            rospy.sleep(1.0)

        # launch_process.terminate()
        # launch_process.wait()
        # kill_processes()
        # print("Finished executing all trajectories.")
