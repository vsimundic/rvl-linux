#!/usr/bin/python

import rospy
import os
import rospkg
from core.read_config import read_config
from DDMan import push
from gazebo_push_open.cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
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
    
    # TXA = cabinet base part w.r.t door (center of the axis)
    _, TXA = generate_cabinet_urdf_from_door_panel(w_door=cabinet_dims['width'], 
                                          h_door=cabinet_dims['height'],
                                          static_d=cabinet_dims['depth'],
                                          save_path=config['cabinet_urdf_save_path'])
    
    
    x = np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x'])
    y = np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y'])
    z = cabinet_dims['height']/2 + 0.018 + 0.01 # half of door height + depth of bottom panel + double static moving part distance
    angle_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])
    TX0 = get_cabinet_world_pose(x, y, z, angle_deg, TXA)
    
    feasible_poses_args = config['feasible_poses']
    feasible_poses = push.demo_push_poses_ros(**feasible_poses_args)
    
    print(feasible_poses)