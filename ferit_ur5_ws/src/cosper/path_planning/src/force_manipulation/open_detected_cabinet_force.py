#!/usr/bin/python
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import rospy
from core.util import read_csv_DataFrame
from core.ur5_commander import UR5Commander
from core.real_ur5_controller import UR5Controller
from core.transforms import rot_z
from gazebo_push_open.cabinet_model import Cabinet
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
from utils import *
# from path_planning.src.force_manipulation.force_utils import *
from force_utils import *
import threading
import json

if __name__ == '__main__':
    rospy.init_node('test_node_simulations')
    
    read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results/results_multi-c_our_handleless_real.csv'
    data = read_csv_DataFrame(read_results_path)

    real_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/config/Exp-real_robot_cabinet_open/results.txt'
    traj_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/config/Exp-real_robot_cabinet_open/trajectories'

    rvl_cfg_path = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

    # Robot handler
    robot = UR5Controller()
    robot.get_current_joint_values()
    T_R_W = np.eye(4)
    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')

    # Load door model
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

    joint_values = np.array(data["joint_values"])
    T_6_0_ros = robot.get_fwd_kinematics(joint_values)

    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.pi)
    T_6_0 = Tz @ T_6_0_ros

    T_A_W = T_R_W @ T_6_0 @ T_C_6 @ T_A_C

    width = s[0]
    height = s[1]
	# Static cabinet params
    door_thickness=0.018
    static_depth=0.4
    push_latch_mechanism_length = 0.046 + door_thickness*0.5
 
    rvl_cfg_path = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

    state_angle = axis_pos*np.rad2deg(np.arcsin(push_latch_mechanism_length/width))

    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                            axis_pos=axis_pos,
                            T_A_S=T_A_W,
                            has_handle=False)

    # Get current joint values and correct them for the path planner
    q_init = robot.get_current_joint_values()
    # adjust joint values from ROS
    q_init[0] += np.pi
    q_init[5] += np.pi
    q_init[q_init>np.pi]-=(2.0*np.pi)     
    q_init[q_init<-np.pi]+=(2.0*np.pi)

    T_G_0_array, q, all_feasible_paths, all_feasible_paths_q = rvl_path_planning(rvl_cfg_path, T_R_W, q_init, 37, False, cabinet_model, state_angle)

    if T_G_0_array.shape[0] < 2:
        print('Path not found.')
        exit(1)

    q[1:, 0] -= np.pi
    q[1:, 5] -= np.pi
    q[q>np.pi] -= (2.0*np.pi)     
    q[q<-np.pi] += (2.0*np.pi)
    np.unwrap(q, axis=0)
    
    # Send the approach trajectory
    monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 30.0))
    monitor_thread.start()
    success = robot.send_joint_trajectory_action(q[:3])        
    monitor_thread.join()

    # Zero the force sensor
    robot.zero_ft_sensor()

    # Send the open trajectory
    monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 30.0))
    monitor_thread.start()
    rospy.sleep(0.05)
    robot.send_joint_trajectory_action(q[2:])
    monitor_thread.join()

    if robot.force_violation:
        T_6_0 = robot.get_current_tool_pose()
        
        T_6offset_6 = np.eye(4)
        T_6offset_6[2, 3] = - 0.1

        T_6goal_0 = T_6_0 @ T_6offset_6

        q = robot.get_closest_ik_solution(T_6goal_0)
        
        if q is None:
            rospy.logwarn('Cannot move the robot back.')
            exit(1)

        # Backup trajectory        
        backup_trajectory = np.array([robot.get_current_joint_values(), q])
        joints_remembered = []
        monitor_thread = threading.Thread(target=monitor_force_drop_and_remember_joints, args=(robot, joints_remembered, 10.0, 1.0, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        robot.send_joint_trajectory_action(backup_trajectory)
        monitor_thread.join()

        print('Remembered joints:', joints_remembered)
    print('Done.')
