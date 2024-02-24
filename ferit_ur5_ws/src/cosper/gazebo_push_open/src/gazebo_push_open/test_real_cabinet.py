#!/usr/bin/python

import rospy
import os
import rospkg

from core.ur5_commander import UR5Commander
from core.rvl import RVLRGBD2PLY
from gazebo_push_open.cabinet_model import Cabinet
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
from core.transforms import rot_z, pose_to_matrix
import RVLPYDDDetector as rvl_dddetector
from trac_ik_python.trac_ik import IK
import yaml
import open3d as o3d
import json
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('node_test_real_cabinet')
    
    # try:
    #     cfg_file = rospy.get_param('config_file')
    # except rospy.exceptions.ROSException:
    #     raise Exception('Could not fetch param.')
    
    # config = read_config(cfg_file)
    

    load = False
    marker_id = 4

    # # Robot handler
    # robot = UR5Commander()

    # Save current robot pose
    joint_values_init = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/real_robot/capture_joint_values.npy')
    T_T_S_init = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/real_robot/capture_T_T_0.npy')
    
    # joint_values_init = robot.get_current_joint_values()
    # T_T_S_init = robot.get_current_tool_pose()
    # np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/real_robot/capture_joint_values.npy', joint_values_init)
    # np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/real_robot/capture_T_T_0.npy', T_T_S_init)

    # robot.send_joint_values_to_robot(joint_values_init)

    sequence_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_open-20240217/sequence_images/2024-02-17-11-08-44'
    rvl_rgbd2ply = RVLRGBD2PLY(sequence_path, 'rgb', 'depth_seg', 'PLY_seg', 'camera_info.yaml')
    # rvl_rgbd2ply.save_plys()

    if not load:

        dd_detector = rvl_dddetector.PYDDDetector()
        dd_detector.create('/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg')
        scene_sequence_dir_name = sequence_path
        ply_dirname = 'PLY_seg'

        for ply_path in rvl_rgbd2ply.ply_paths:
            dd_detector.add_mesh(ply_path)

        for rgb_path in rvl_rgbd2ply.rgb_paths:
            dd_detector.add_rgb(rgb_path)

        ao = dd_detector.detect()
        print(ao)

        ao_copy = ao.copy()
        ao_copy['R'] = ao['R'].tolist()
        ao_copy['t'] = ao['t'].tolist()
        ao_copy['s'] = ao['s'].tolist()
        ao_copy['r'] = ao['r'].tolist()
        with open(os.path.join(sequence_path, 'ao.yaml'), 'w') as f:
            yaml.dump(ao_copy, f)
            print('Saved.')
    else:
        with open(os.path.join(sequence_path, 'ao.yaml'), 'r') as f:
            try:
                ao = yaml.safe_load(f)
                for key in ao:
                    if isinstance(ao[key], list):
                        ao[key] = np.array(ao[key])
            except yaml.YAMLError as exc:
                print(exc)
    
    print(ao)



    # Create a cabinet
    cabinet_door_dims = [ao['s'][0], ao['s'][1], 0.018, 0.4] # w_door, h_door, d_door, static_depth 

    T_A_C = np.eye(4)
    T_A_C[:3, :3] = ao['R']
    T_A_C[:3, 3] = ao['t']

    T_V_A = np.eye(4)
    T_V_A[:2, 3] = ao['r']

    T_D_V = np.eye(4)
    T_D_V[:3, 3] = np.array([-0.018/2., -ao['s'][0]/2., ao['s'][1]/2.])

    T_D_A = T_V_A @ T_D_V

    T_C_T = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_C_T.npy')
    # T_A_S = T_T_S_init @ robot.T_C_T @ T_A_C
    T_A_S = T_T_S_init @ T_C_T @ T_A_C

    T_D_S = T_A_S @ T_D_A


    T_G_T = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_G_T_pen.npy')    
    # T_G_T = np.eye(4)
    # T_G_T[:3, 3] = np.array([0.0025802619431676845, 0.0055984724884192, 0.29844796866823414])

    # Compare SA to marker 4
    with open('/home/RVLuser/ferit_ur5_ws/src/other/calibration_program/data/markers/marker%d.json' %marker_id) as f:
        data = json.load(f)
        pose = Pose()
        pose.position.x = data["position"]["x"]
        pose.position.y = data["position"]["y"]
        pose.position.z = data["position"]["z"]
        pose.orientation.x = data["orientation"]["x"]
        pose.orientation.y = data["orientation"]["y"]
        pose.orientation.z = data["orientation"]["z"]
        pose.orientation.w = data["orientation"]["w"]


        T_T4_S = pose_to_matrix(pose)
        T_M4_S = T_T4_S @ T_G_T
        print("Loaded marker {} from file:".format(id))

    print('T_A_S - T_M4_S:')
    print(T_A_S - T_M4_S)
    pass
    # T_T_S = robot.get_current_tool_pose()
    # T_M4_S = T_T_S @ T_G_T
    # print('T_A diff:\n', T_A_S - T_M4_S)


    # # Create a cabinet object
    # cabinet_model = Cabinet(door_params=np.array(cabinet_door_dims), 
    #                         axis_pos=int(ao['openingDirection']),
    #                         # axis_pos=1,
    #                         T_A_S=T_A_S,
    #                         save_path=None)

    # T_D_S = cabinet_model.T_O_S @ cabinet_model.T_A_O @ cabinet_model.T_D_A


    
    # joint_values = robot.get_current_joint_values()
    # T_T_S = robot.get_current_tool_pose()

    # T_G_S = T_T_S @ T_G_T

    # print(T_G_S - T_D_S)


    # cabinet_model.visualize(T_T_S)
