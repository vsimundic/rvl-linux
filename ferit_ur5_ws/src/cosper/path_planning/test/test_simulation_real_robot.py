#!/usr/bin/python

import rospy
import os
import rospkg
from core.util import read_config, read_csv_DataFrame
from core.paths_packages import get_package_name_from_node, get_package_path_from_name
from core.ur5_commander import UR5Commander
from core.transforms import rot_z
from DDMan import push
from gazebo_push_open.cabinet_model import Cabinet
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
import RVLPYDDManipulator as rvlpy_dd_man
from trac_ik_python.trac_ik import IK
import roslaunch
from gazebo_msgs.msg import ContactsState
from subprocess import check_output
import signal
import csv


if __name__ == '__main__':
    rospy.init_node('test_node_simulations')
    
    read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results.csv'
    data = read_csv_DataFrame(read_results_path)

    success_data = data.loc[((data['path_found'] == True) & 
                            (data['traj_success'] == True) & 
                            (data['contact_free'] == True) & 
                            (data['door_opened'] == True))] 
    print(success_data)

    n = 10
    T_G_T_pen = np.eye(4)
    T_G_T_pen[2, 3] = 0.310

    # Robot handler
    robot = UR5Commander()  
    successful_exps = []
    for i, row in success_data.head(n).iterrows():
        w = row['door_width']
        h = row['door_height']
        x = row['x']
        y = row['y']
        z = row['z']
        
        rz = np.radians(row['rot_z'])
        axis_pos = row['axis_pos']

        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array([x, y, z])
        Tz_init = np.eye(4)
        Tz_init[:3, :3] = rot_z(np.radians(90.))
        T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(rz)
        T_A_S = T_A_S @ Tz
        print(T_A_S)

        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array([w, h, 0.018, 0.4]), 
                                axis_pos=axis_pos,
                                T_A_S=T_A_S)
        
        T_pt1_A = np.eye(4)
        T_pt1_A[:3, 3] = np.array([-0.009, cabinet_model.axis_distance, h*0.5])
        T_pt2_A = np.eye(4)
        T_pt2_A[:3, 3] = np.array([-0.009, cabinet_model.axis_distance - w, h*0.5])

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

        while True:
            key = input('Press p when done placing the cabinet:')
            if key == 'p':
                break
        

            
        path_planner = rvlpy_dd_man.PYDDManipulator()
        path_planner.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec.cfg')
        path_planner.set_robot_pose(robot.T_B_S)
        path_planner.set_door_model_params(
                                        cabinet_model.d_door,
                                        cabinet_model.w_door,
                                        cabinet_model.h_door,
                                        0.0, # rx
                                        -(cabinet_model.w_door/2. - cabinet_model.axis_distance), # ry
                                        cabinet_model.axis_pos, # opening direction
                                        cabinet_model.static_side_width,
                                        cabinet_model.moving_to_static_part_distance) 
        path_planner.set_door_pose(T_A_S)
        path_planner.set_environment_state(-8.0)

        # Get current joint values and correct them for the path planner
        q_init = robot.get_current_joint_values()
        q_init = np.array(q_init)
        # adjust joint values from ROS
        q_init[0] += np.pi
        q_init[5] += np.pi
        q_init[q_init>np.pi]-=(2.0*np.pi)     
        q_init[q_init<-np.pi]+=(2.0*np.pi)

        # Plan 
        T_G_0_array, q = path_planner.path2(np.array(q_init))

        if T_G_0_array.shape[0] == 1:
            print('Path not found.')

        del path_planner

        q[:, 0] += np.pi
        q[:, 5] += np.pi
        q[q>np.pi]-=(2.0*np.pi)     
        q[q<-np.pi]+=(2.0*np.pi)

        robot.generate_URScript(q, with_force_mode=False)
        robot.send_URScript()

        key = input('press 1 or 0 to store experiment success:')
        successful_exps.append(int(key))
        print(successful_exps)