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

if __name__ == '__main__':
    rospy.init_node('node_generate_cabinet')
    
    try:
        cfg_file = rospy.get_param('config_file')
    except rospy.exceptions.ROSException:
        raise Exception('Could not fetch param.')
    
    config = read_config(cfg_file)
    
    cabinet_dims = config['cabinet_dims']
    cabinet_pose = config['cabinet_pose']


    cabinet_model = Cabinet(w_door=cabinet_dims['width'], 
                            h_door=cabinet_dims['height'],
                            d_door=0.018,
                            static_d=cabinet_dims['depth'],
                            axis_pos=cabinet_pose['axis_pos'], # axis on the right side
                            save_path=config['cabinet_urdf_save_path'])
        
    x = np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x'])
    y = np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y'])
    z = cabinet_dims['height']/2 + 0.018 + 0.01 # half of door height + depth of bottom panel + double static moving part distance
    angle_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])
    
    TX0 = cabinet_model.get_world_pose(x, y, z, angle_deg)

    cabinet_model.delete_model_gazebo()
    cabinet_model.spawn_model_gazebo(x, y, z, angle_deg)

    theta_deg = config['feasible_poses']['dd_state_deg']

    cabinet_model.move_model_joint_gazebo(theta_deg)

    feasible_poses_args = config['feasible_poses']
    feasible_poses = push.demo_push_poses_ros(**feasible_poses_args)
    
    TGX = cabinet_model.get_gripper_pose_from_feasible_pose(feasible_poses[0])
    TB0 = np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TB0.npy'))
    TGT = np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TGT.npy'))

    TTB = TB0.T @ TX0 @ TGX @ TGT.T

    robot = UR5Commander()
    robot.send_pose_to_robot(TTB)




