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

if __name__ == '__main__':
    rospy.init_node('test_node_simulations')
    
    read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results/results_multi-c_our_handleless_real.csv'
    data = read_csv_DataFrame(read_results_path)

    real_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/config/Exp-real_robot_cabinet_open/results.txt'
    traj_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/config/Exp-real_robot_cabinet_open/trajectories'

    START_FROM_BEGGINING = False
    
    exp_success = []
    
    i_ = 0
    if not START_FROM_BEGGINING:
        exp_success = np.loadtxt(real_results_path, delimiter=',',dtype=int).tolist()
        if type(exp_success) is not list:
            exp_success = [exp_success]
        i_ = len(exp_success)
        print(exp_success)
    
    # rvl_cfg_path = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

	# Load door configurations
    door_configs_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/cabinet_configurations_axis_left_real.npy'
    doors = np.load(door_configs_path)
    num_doors = int(doors.shape[0]*0.5)

    success_data = data.loc[((data['path_found'] == True) & 
                            (data['traj_success'] == True) & 
                            (data['contact_free'] == True) & 
                            (data['door_opened'] == True))] 
    print(success_data)

    n = 50
    T_G_T_pen = np.eye(4)
    T_G_T_pen[2, 3] = 0.313

    T_R_W = np.eye(4)

	# Static cabinet params
    door_thickness=0.018
    static_depth=0.4

    # Robot handler
    # robot = UR5Commander()  
    robot = UR5Controller()

    # exps = list(success_data.head(n).iterrows())
    exps = success_data.head(n).axes[0].tolist()
    # i_ = 0
    for i_idx in range(i_, len(exps)):
    # for i in range(num_doors):
        # i_idx = 37
        i = exps[i_idx]
        door = doors[i, :]
        width = door[0]
        height = door[1]
        position = door[2:5]
        rot_z_deg = door[5]
        state_angle = door[6]
        axis_pos = door[7]

        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(position)
        # Tz_init = np.eye(4)
        # Tz_init[:3, :3] = rot_z(np.radians(90.))
        # T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
        T_A_S = T_A_S @ Tz


        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                                axis_pos=axis_pos,
                                T_A_S=T_A_S,
                                has_handle=False)

        T_pt1_A = np.eye(4)
        T_pt1_A[:3, 3] = np.array([-0.009, 0, height*0.5])
        T_pt2_A = np.eye(4)
        T_pt2_A[:3, 3] = np.array([-0.009, -width, height*0.5])

        Tz_poly = np.eye(4)
        Tz_poly[:3, :3] = rot_z(np.radians(180.))

        T_G1_S = T_A_S @ T_pt1_A
        T_G1_S_poly = Tz_poly @ T_G1_S
        print(T_G1_S_poly[:3,3]*1000)
        # print(T_G1_S @ Tz_poly)
        T_G2_S = T_A_S @ T_pt2_A
        T_G2_S_poly = Tz_poly @ T_G2_S
        print(T_G2_S_poly[:3,3]*1000)
        # print(T_G2_S @ Tz_poly)

        # Get current joint values and correct them for the path planner
        q_init = robot.get_current_joint_values()
        # q_init = np.array(q_init)
        # adjust joint values from ROS
        q_init[0] += np.pi
        q_init[5] += np.pi
        q_init[q_init>np.pi]-=(2.0*np.pi)     
        q_init[q_init<-np.pi]+=(2.0*np.pi)

        # T_G_0_array, q, all_feasible_paths, all_feasible_paths_q = rvl_path_planning(rvl_cfg_path, T_R_W, q_init, 37, False, cabinet_model, state_angle)

        # if T_G_0_array.shape[0] == 1:
        #     print('Path not found.')
        #     print('cabinet %d' %i)
        #     continue

        traj_filename = os.path.join(traj_path, 'traj_%d.txt' % i)

        q = np.loadtxt(traj_filename, delimiter=',')

        while True:
            key = input('Press p when done placing the cabinet:')
            if key == 'p':
                break

        monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 30.0))
        monitor_thread.start()
        success = robot.send_joint_trajectory_action(q[:3])        
        monitor_thread.join()

        robot.zero_ft_sensor()

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
                break
            
            backup_trajectory = np.array([robot.get_current_joint_values(), q])

            monitor_thread = threading.Thread(target=monitor_force_drop_and_remember_joints, args=(robot, 10.0, 1.0, 50.0))
            monitor_thread.start()
            rospy.sleep(0.05)
            robot.send_joint_trajectory_action(backup_trajectory)
            monitor_thread.join()

        while True:
            key = input('Press s when robot finishes:')
            if key == 's':
                break

        key = input('press 1 or 0 to store experiment success:')
        exp_success.append(int(key))

        # np.savetxt(real_results_path, np.array(exp_success, dtype=int), delimiter=',')
        print(exp_success)