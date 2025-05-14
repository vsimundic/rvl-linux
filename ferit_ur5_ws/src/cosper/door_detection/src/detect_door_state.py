#!/usr/bin/env python

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rosnode import kill_nodes
from PIL import Image as PilImage
import RVLPYRGBD2PLY as rvl
import RVLPYDDDetector
import json 
from core.real_ur5_controller import UR5Controller
from core.ur5_commander import UR5Commander
import core.transforms as tfs

def main():
    # Initialize ROS node
    rospy.init_node("door_state_detection_node", anonymous=True)
    node_name = rospy.get_name()
    rospy.loginfo(f"Node name: {node_name}")
    
    # Door model parameters
    base_dir = rospy.get_param("~base_dir", "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection")
    rgb_dir = rospy.get_param("~rgb_dir", "RGB_images")
    depth_dir = rospy.get_param("~depth_dir", "DEPTH_images")
    ply_dir = rospy.get_param("~ply_dir", "PLY_seg")
    model_output_path = rospy.get_param("~model_output_path", "models/doorModel.json")
    detector_config_path = rospy.get_param("~detector_config_path", "/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg")

    model_output_path = os.path.join(base_dir, model_output_path)
    best_hyp_path = os.path.join(base_dir, "DDT.txt")

    # Load door model from json
    with open(model_output_path, 'r') as f:
        door_model = json.load(f)
    
    # Different angle state parameters
    diff_base_dir = rospy.get_param("~diff_base_dir", "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection")
    diff_rgb_dir = rospy.get_param("~diff_rgb_dir", "RGB_images")
    diff_depth_dir = rospy.get_param("~diff_depth_dir", "DEPTH_images")
    diff_ply_dir = rospy.get_param("~diff_ply_dir", "PLY_seg")
    image_number = rospy.get_param("~image_number", 45)
    diff_model_output_path = rospy.get_param("~diff_model_output_path", "models/doorModel.json")
    diff_model_output_path = os.path.join(diff_base_dir, diff_model_output_path)

    # Image and mesh path
    # rgb_path = os.path.join(diff_base_dir, diff_rgb_dir, f"{image_number:04d}.png")
    rgb_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/detect_state_images/rgb/captured_rgb.png'
    # ply_path = os.path.join(diff_base_dir, diff_ply_dir, f"{image_number:04d}.ply")
    ply_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/detect_state_images/ply/captured.ply'

    # Load different angle door model from json
    with open(diff_model_output_path, 'r') as f:
        diff_door_model = json.load(f)

    # Initialize detector for state estimation
    detector = RVLPYDDDetector.PYDDDetector()
    detector.create(detector_config_path)
    detector.load_best_hypothesis(best_hyp_path)

    # Initialize UR5 controller
    # robot = UR5Commander()
    robot = UR5Controller()

    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')

    q_detected = door_model["joint_values"]
    q_detected = np.array(q_detected)
    # T_6_0_detected = robot.forward_kinematics(q_detected)
    T_6_0_detected = robot.get_fwd_kinematics_moveit(q_detected.tolist())
    T_C_0_detected = T_6_0_detected @ T_C_6

    # Different angle
    # q_diff = diff_door_model["joint_values"]
    q_diff = robot.get_current_joint_values()
    q_diff = np.array(q_diff)
    T_6_0_diff = robot.get_fwd_kinematics_moveit(q_diff.tolist())
    T_C_0_diff = T_6_0_diff @ T_C_6

    T_Cdet_Cdiff = np.linalg.inv(T_C_0_diff) @ T_C_0_detected
    # T_Cdet_Cdiff = np.linalg.inv(T_C_0_detected) @ T_C_0_diff

    detector.add_mesh(ply_path)
    detector.add_rgb(rgb_path)

    states = detector.recognize_ao_state(T_Cdet_Cdiff, True)
    print("Detected states:", np.rad2deg(states))


if __name__ == "__main__":
    main()