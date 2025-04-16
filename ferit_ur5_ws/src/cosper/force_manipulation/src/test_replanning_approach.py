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
from core.real_ur5_controller import UR5Controller
from core.transforms import rot_z
from gazebo_push_open.cabinet_model import Cabinet
from core.gazebo import get_link_pose
from force_utils import *
from push_force_trajectories import generate_tool_line_poses, generate_trajectories, generate_trajectories_and_approach
sys.path.append(os.path.join(os.path.dirname(__file__), '../../path_planning/src'))
from utils import contact_callback, kill_processes, reset_tf_buffer
import RVLPYDDManipulator as rvlpy
import threading

"""
This script gets a ground truth door model from a pen on the robot. Then it detects the door model and
generates an approach path with a trajectory. If the approach path is not successful, it will replan the trajectory
and try again with the approach path from the ground truth door model.
"""


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

    T_0_W = np.eye(4)
    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')

    # RVL manipulator to get forward kinematics
    rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_cfg)
    rvl_manipulator.set_robot_pose(T_0_W)

    T_G_6 = rvl_manipulator.get_T_G_6()

    # UR5 controller
    robot = UR5Controller()

    T_A_W_gt = np.eye(4)
    width = 0.396
    height = 0.496
    door_thickness = 0.018
    static_depth = 0.4
    push_latch_mechanism_length = 0.044 + door_thickness * 0.5
    axis_pos = -1
    state_angle_gt = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / width))

    load_gt = True
    gt_door_model_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/models/T_A_W_gt.npy'
    if load_gt:
        # Load ground truth door model from file
        T_A_W_gt = np.load(gt_door_model_path)
    else:
        # Calculate the ground truth door model from the points on the front panel
        T_pen_6 = np.eye(4)
        T_pen_6[2, 3] = 0.314 # length of the pen

        T_6_0_pts = np.zeros((2, 4, 4))
        T_6_0_pts[:, 3, 3] = 1.

        for i in range(2):
            print("Place the robot in point %d." % i)
            input("Press Enter to continue...")

            T_6_0 = robot.get_current_tool_pose()
            T_6_0_pts[i, :, :] = T_6_0.copy()    
        
        # T_6_0_pts = np.array([[[ 0.74216537, -0.60411914, -0.29022513, -0.13162687],
        #         [ 0.07600705, -0.35437219,  0.93201034,  0.27084308],
        #         [-0.66589301, -0.71376496, -0.21708545,  0.57335662],
        #         [ 0.        ,  0.        ,  0.        ,  1.        ]],

        #     [[ 0.63323669, -0.65144094,  0.41789471,  0.04091646],
        #         [-0.59003294, -0.05690475,  0.80537133,  0.28637176],
        #         [-0.50087166, -0.75656232, -0.42040579,  0.63628171],
        #         [ 0.        ,  0.        ,  0.        ,  1.        ]]])
        T_pts_0 = T_6_0_pts @ T_pen_6[np.newaxis, ...]
        
        # S_0 = S_W
        # Rotation of T_A_W
        z_ = np.array([0, 0, 1])
        y_ = T_pts_0[0, :3, 3] - T_pts_0[1, :3, 3]
        y_ = np.array(y_ / np.linalg.norm(y_))
        x_ = np.cross(y_, z_)
        T_A_W_gt[:3, 1] = y_.copy()
        T_A_W_gt[:3, 2] = z_.copy()
        T_A_W_gt[:3, 0] = x_.copy()

        # translation of T_A_W
        T_A_W_gt[:3, 3] = T_pts_0[0, :3, 3].copy()
        T_A_W_gt[:3, 3] = T_A_W_gt[:3, 0]*door_thickness*0.5 + T_A_W_gt[:3, 3]
        T_A_W_gt[:3, 3] = -T_A_W_gt[:3, 2]*height*0.5 + T_A_W_gt[:3, 3]

        np.save(gt_door_model_path, T_A_W_gt)

        exit(0)

    cabinet_gt = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]),
                            r=np.array([0, -width*0.5]),
                            axis_pos=axis_pos,
                            T_A_S=T_A_W_gt,
                            save_path=cabinet_urdf_save_path,
                            has_handle=False)

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
    state_angle = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / s[0]))
    joint_values = np.array(data["joint_values"])
    T_6_0 = robot.get_fwd_kinematics_moveit(joint_values)

    T_A_W = T_0_W @ T_6_0 @ T_C_6 @ T_A_C

    cabinet_model = Cabinet(door_params=np.array([s[0], s[1], door_thickness, static_depth]),
                            r=r,
                            axis_pos=axis_pos,
                            T_A_S=T_A_W,
                            save_path=cabinet_urdf_save_path,
                            has_handle=False)

    # rvl_manipulator.set_door_model_params(
    #     cabinet_model.d_door,
    #     cabinet_model.w_door,
    #     cabinet_model.h_door,
    #     cabinet_model.rx,
    #     cabinet_model.ry,
    #     cabinet_model.axis_pos,
    #     cabinet_model.static_side_width,
    #     cabinet_model.moving_to_static_part_distance)
    # rvl_manipulator.set_door_pose(cabinet_model.T_A_S)
    # rvl_manipulator.set_environment_state(state_angle)

    # Generate poses and trajectories
    R_TCP_D = np.array([[0, 0, -1], 
                        [0, 1, 0], 
                        [1, 0, 0]])
    T_TCP_G = np.eye(4)
    T_TCP_G[:3, 3] = np.array([0.155 * 0.5, 0, 0.098])
    T_G_DD_all = generate_tool_line_poses(height, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)

    # Debug trajectories with approach
    _, trajectories = generate_trajectories_and_approach(T_G_DD_all, 37, state_angle, -90.0, cabinet_model, rvl_cfg, T_0_W, robot)
    _, trajectories_gt = generate_trajectories_and_approach(T_G_DD_all, 37, state_angle_gt, -90.0, cabinet_gt, rvl_cfg, T_0_W, robot)

    if len(trajectories) < 1:
        print("No detected model trajectories generated.")
        exit(0)
    elif len(trajectories_gt) < 1:
        print("No ground truth trajectories generated.")
        exit(0)
        
    print(f"Generated {len(trajectories)} trajectory roots.")
    print(f"Generated {len(trajectories_gt)} ground truth trajectory roots.")

    trajectory = trajectories[0]
    if not robot.validate_trajectory_points(trajectory):
        print("Trajectory is not valid.")
        exit(0)

    trajectory[:, 0] -= np.pi
    trajectory[:, 5] -= np.pi
    trajectory[trajectory > np.pi] -= 2 * np.pi
    trajectory[trajectory < -np.pi] += 2 * np.pi
    trajectory = np.unwrap(trajectory, axis=0)

    # Create the approach path
    approach_path = np.array([robot.get_current_joint_values(), trajectory[0], trajectory[1]])

    # Send the approach path to the robot
    monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 30.0))
    monitor_thread.start()
    rospy.sleep(0.05)
    robot.send_joint_trajectory_action(approach_path, max_velocity=0.5, max_acceleration=0.5)
    monitor_thread.join()

    # Zero the force sensor
    robot.zero_ft_sensor()

    # Send the insertion trajectory
    insertion_trajectory = np.array([robot.get_current_joint_values(), trajectory[2]])
    monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 8.0))
    monitor_thread.start()
    success = robot.send_joint_trajectory_action(insertion_trajectory, max_velocity=0.5, max_acceleration=0.5)
    monitor_thread.join()

    if not success:
        print("Insertion trajectory failed.")
        # Backup trajectory
        backup_trajectory = np.array([robot.get_current_joint_values(), trajectory[1]])
        robot.send_joint_trajectory_action(backup_trajectory, max_velocity=0.5, max_acceleration=0.5)

        # Take trajectory from gt
        trajectory = trajectories_gt[0]
        trajectory[:, 0] -= np.pi
        trajectory[:, 5] -= np.pi
        trajectory[trajectory > np.pi] -= 2 * np.pi
        trajectory[trajectory < -np.pi] += 2 * np.pi
        trajectory = np.unwrap(trajectory, axis=0)

        if not robot.validate_trajectory_points(trajectory):
            print("GT trajectory is not valid.")
            exit(0)

        # Create the reroute path before insertion
        reroute_trajectory = np.array([robot.get_current_joint_values(), trajectory[1], trajectory[2]])
        monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 20.0))
        monitor_thread.start()
        success = robot.send_joint_trajectory_action(reroute_trajectory, max_velocity=0.5, max_acceleration=0.5)
        monitor_thread.join()

        if not success:
            print("Reroute trajectory failed.")
            exit(0)
        
        # Zero the force sensor
        robot.zero_ft_sensor()
        
    # Send the open trajectory
    monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 30.0))
    monitor_thread.start()
    rospy.sleep(0.05)
    robot.send_joint_trajectory_action(trajectory[2:], max_velocity=0.5, max_acceleration=0.5)
    monitor_thread.join()
