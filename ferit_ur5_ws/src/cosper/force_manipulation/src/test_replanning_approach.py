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
from core.transforms import rot_z, rot_y, rot_x, rodrigues_rotate_vector
from gazebo_push_open.cabinet_model import Cabinet
from core.gazebo import get_link_pose
from force_utils import *
# from push_force_trajectories import generate_tool_line_poses, generate_trajectories, generate_trajectories_and_approach
from push_force_trajectories import *
sys.path.append(os.path.join(os.path.dirname(__file__), '../../path_planning/src'))
from utils import contact_callback, kill_processes, reset_tf_buffer
import RVLPYDDManipulator as rvlpy
import threading
import open3d as o3d
sys.path.append(os.path.join(os.path.dirname(__file__), '../../door_detection/src'))
from door_state_detector import DoorStateDetector
from core.image_process import OneShotImageCapture

"""
This script gets a ground truth door model from a pen on the robot. Then it detects the door model and
generates an approach path with a trajectory. If the approach path is not successful, it will replan the trajectory
and try again with the approach path from the ground truth door model.
"""

door_thickness = 0.018
static_depth = 0.4
push_latch_mechanism_length = 0.046 + door_thickness * 0.5
opening_angle = -45.0


def create_gt_cabinet_model(load_gt: bool, 
                            gt_door_model_path: str, 
                            gt_door_width_path: str, 
                            cabinet_urdf_save_path: str,
                            robot: Union[UR5Commander, UR5Controller]):
    global door_thickness, static_depth, push_latch_mechanism_length
    T_A_W_gt = np.eye(4)
    width = 0.396
    height = 0.496
    door_thickness = 0.018
    static_depth = 0.4
    axis_pos = -1

    if load_gt:
        # Load ground truth door model from file
        T_A_W_gt = np.load(gt_door_model_path)
        width = np.load(gt_door_width_path)
    else:
        # Calculate the ground truth door model from the points on the front panel
        T_pen_6 = np.eye(4)
        T_pen_6[2, 3] = 0.31182 # length of the pen

        T_6_0_pts = np.zeros((2, 4, 4))
        T_6_0_pts[:, 3, 3] = 1.

        for i in range(2):
            print("Place the robot in point %d." % i)
            input("Press Enter to continue...")

            T_6_0 = robot.get_current_tool_pose()
            T_6_0_pts[i, :, :] = T_6_0.copy()

        T_pts_0 = T_6_0_pts @ T_pen_6[np.newaxis, ...]

        # S_0 = S_W
        # Rotation of T_A_W
        z_ = np.array([0, 0, 1])
        y_ = T_pts_0[0, :3, 3] - T_pts_0[1, :3, 3]
        width = np.linalg.norm(y_)
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
        np.save(gt_door_width_path, width)

        exit(0)

    state_angle_gt = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / width))
    cabinet_gt = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]),
                            r=np.array([0, -width*0.5]),
                            axis_pos=axis_pos,
                            T_A_S=T_A_W_gt,
                            save_path=cabinet_urdf_save_path,
                            has_handle=False)
    return cabinet_gt, state_angle_gt

def create_detected_cabinet_model(data_model_path: str, 
                                  robot: Union[UR5Commander, UR5Controller]):
        # Load door model info
    with open(data_model_path, 'r') as f:
        data = json.load(f)

    R = np.array(data["R"])
    t = np.array(data["t"])
    s = np.array(data["s"])
    r = np.array(data["r"])
    axis_pos = data["openingDirection"]
    T_A_C = np.eye(4)
    T_A_C[:3, :3] = R
    T_A_C[:3, 3] = t
    # state_angle = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / s[0]))
    state_angle = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / s[0]))
    joint_values = np.array(data["joint_values"])

    if type(robot) is UR5Controller:
        T_6_0 = robot.get_fwd_kinematics_moveit(joint_values)
    else:
        T_6_0 = robot.forward_kinematics(joint_values)

    T_A_W = T_0_W @ T_6_0 @ T_C_6 @ T_A_C
    cabinet_model = Cabinet(door_params=np.array([s[0], s[1], door_thickness, static_depth]),
                            r=r,
                            axis_pos=axis_pos,
                            T_A_S=T_A_W,
                            save_path=cabinet_urdf_save_path,
                            has_handle=False)
    
    return cabinet_model, T_6_0, state_angle


if __name__ == '__main__':
    rospy.init_node('single_cabinet_custom_trajectory_node')

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    # Static cabinet mesh filenames
    base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'
    cabinet_static_mesh_filename = os.path.join(base_dir, 'cabinet_model/cabinet_static_mesh.ply')
    cabinet_panel_mesh_filename = os.path.join(base_dir, 'cabinet_model/cabinet_panel_mesh.ply')
    cabinet_full_mesh_filename = os.path.join(base_dir, 'cabinet_model/cabinet_full_mesh.ply')
    cabinet_urdf_save_path = os.path.join(base_dir, 'cabinet_model/cabinet_model.urdf')
    trajs_path = os.path.join(base_dir, 'trajectories.npy')
    
    cabinet_models_path = os.path.join(base_dir, 'cabinet_model')
    if not os.path.exists(cabinet_models_path):
        os.makedirs(cabinet_models_path)

    gt_door_model_path = os.path.join(base_dir, 'models/T_A_W_gt.npy')
    gt_door_width_path = os.path.join(base_dir, 'models/width.npy')
    door_model_path = os.path.join(base_dir, 'models/doorModel.json')

    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')
    T_0_W = np.eye(4)

    # Image capture parameters
    capture_dir = os.path.join(base_dir, 'detect_state_images')
    save_dir = capture_dir
    rgb_topic = '/camera/color/image_raw'
    depth_topic = '/camera/aligned_depth_to_color/image_raw'
    camera_info_topic = '/camera/color/camera_info'
    depth_encoding = '16UC1'  # '16UC1' or '32FC1'

    # Door detector config path
    door_detector_config_path = '/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg'
    best_hyp_path = os.path.join(base_dir, 'DDT.txt')

    # RVL manipulator to get forward kinematics
    rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_cfg)
    rvl_manipulator.set_robot_pose(T_0_W)

    T_G_6 = rvl_manipulator.get_T_G_6()

    # UR5 controller
    robot = UR5Controller()
    robot.T_C_6 = T_C_6
    robot.T_G_6 = T_G_6
    # robot = UR5Commander()

    load_gt = False
    cabinet_gt, state_angle_gt = create_gt_cabinet_model(load_gt, gt_door_model_path, gt_door_width_path, cabinet_urdf_save_path=cabinet_urdf_save_path, robot=robot)
    cabinet_model, T_6_0, state_angle = create_detected_cabinet_model(door_model_path, robot)

    # test - get camera pose on sphere 
    # T_C_W_new, joints = get_camera_pose_on_sphere_distance(cabinet_model, robot, rvl_manipulator, T_6_0, state_angle)

    # Generate poses and trajectories
    R_TCP_D = np.array([[0, 0, -1],
                        [0, 1, 0],
                        [1, 0, 0]])
    T_TCP_G = np.eye(4)
    # T_TCP_G[:3, 3] = np.array([0.155 * 0.5, 0, 0.098])
    T_TCP_G[:3, 3] = np.array([0.155 * 0.5, 0, 0.098])
    T_G_DD_all = generate_tool_line_poses(cabinet_model.h_door, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)

    # Debug trajectories with approach
    _, trajectories = generate_trajectories_and_approach2(T_G_DD_all, 18, state_angle, opening_angle, cabinet_model, rvl_cfg, T_0_W, robot)
    _, trajectories_gt = generate_trajectories_and_approach2(T_G_DD_all, 18, state_angle_gt, opening_angle, cabinet_gt, rvl_cfg, T_0_W, robot)    

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


    # Send the opening trajectory and monitor the force on the tactile sensor
    remembered_joints = []
    monitor_thread = threading.Thread(
        target=monitor_tactile_loss_and_remember_joints, 
        args=(robot, remembered_joints, 0.01)
    )
    monitor_thread.start()
    success = robot.send_joint_trajectory_action(trajectory[2:], max_velocity=0.5, max_acceleration=0.5)
    monitor_thread.join()

    if not success and len(remembered_joints) > 0:
        # Take the first remembered joint (there should usually be only one)
        remembered_q = np.array(remembered_joints[0])
        
        # Find the closest point in the trajectory
        cheb_distances = np.max(np.abs(trajectory - remembered_q), axis=1)
        closest_idx = np.argmin(cheb_distances)

        rospy.loginfo(f"[replan] Closest trajectory point index: {closest_idx}")

        # Calculate door angle
        door_opening_angles = np.linspace(state_angle, opening_angle, trajectory[2:].shape[0])
        estimated_door_angle = door_opening_angles[closest_idx]

        print(f"[replan] Estimated door angle at contact loss: {estimated_door_angle:.2f} degrees")

        # Generate new camera pose for capturing the door state
        T_C_W_new, joints_camera_capture = get_camera_pose_on_sphere_distance(cabinet_model, robot, rvl_manipulator, T_6_0, estimated_door_angle)
        # Check closest joints configuration
        joints_camera_capture_closest = joints_camera_capture[np.argmin(np.linalg.norm(joints_camera_capture - remembered_q, axis=1))]
        
        # Send the robot to the closest joint configuration, but with the backup trajectory
        T_6_0_current = robot.get_current_tool_pose()
        # Plan backup - in negative z-direction
        T_6_0_backup = T_6_0_current.copy()
        T_6_0_backup[:3, 3] = T_6_0_backup[:3, 3] - T_6_0_backup[:3, 2] * 0.1 # move 10 cm in z-direction
        joints_backup, _ = robot.get_closest_ik_solution(T_6_0_backup, None) # get ik solution for backup and take current joints
        # Send the robot to the backup position and then to the camera capture position
        robot.send_joint_trajectory_action(np.array([robot.get_current_joint_values(), joints_backup, joints_camera_capture_closest]), max_velocity=0.5, max_acceleration=0.5)

        # Capture the door state
        image_capture = OneShotImageCapture(save_dir, rgb_topic, depth_topic, camera_info_topic, depth_encoding)
        rgb_path, depth_path, ply_path = image_capture.capture_single_image_and_save()

        # Transform from detection capture to state capture
        # Load doorModel from json and get joint_values
        with open(door_model_path, 'r') as f:
            data = json.load(f)
        q_detected = data["joint_values"]
        q_detected = np.array(q_detected)
        T_6_0_detected = robot.forward_kinematics(q_detected)
        T_C_W_detected = robot.T_0_W @ T_6_0_detected @ T_C_6

        T_Cdet_Cdiff = np.linalg.inv(T_C_W_detected) @ T_C_W_new

        # Detect the door state
        door_state_detector = DoorStateDetector(detector_config_path=door_detector_config_path, best_hyp_path=best_hyp_path)
        state_deg = door_state_detector.detect_state(rgb_path, ply_path, T_Cdet_Cdiff)
        print(f"[replan] Detected door state: {state_deg}")

        # Plan approach and trajectory
        _, trajectories = generate_trajectories_and_approach2(T_G_DD_all, 18, state_deg, opening_angle, cabinet_model, rvl_cfg, T_0_W, robot)
        if len(trajectories) < 1:
            print("No detected model trajectories generated.")
            exit(0)
        # Send the approach path to the robot
        trajectory = trajectories[0]
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

        # Open the door
        if success:
            print("[replan] Insertion trajectory successful.")
            # Send the opening trajectory and monitor the force on the tactile sensor
            remembered_joints = []
            monitor_thread = threading.Thread(
                target=monitor_tactile_loss_and_remember_joints, 
                args=(robot, remembered_joints, 0.01)
            )
            monitor_thread.start()
            success = robot.send_joint_trajectory_action(trajectory[2:], max_velocity=0.5, max_acceleration=0.5)
            monitor_thread.join()

            if not success:
                print("[replan] Opening trajectory failed.")
                exit(0)
        else:
            print("[replan] Insertion trajectory failed.")
            exit(0)

    else:
        print("[replan] No contact loss detected during opening.")
