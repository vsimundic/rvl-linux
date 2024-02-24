#!/usr/bin/python

import rospy
from core.rvl import RVLRGBD2PLY
from core.read_config import read_config

if __name__ == '__main__':
    rospy.init_node('node_test_real_cabinet')
    
    try:
        cfg_file = rospy.get_param('config_file')
    except rospy.exceptions.ROSException:
        raise Exception('Could not fetch param.')
    
    config = read_config(cfg_file)
    

    sequence_path = config['sequence_path']
    rgb_dir = config['rgb_dir']
    depth_dir = config['depth_dir']
    ply_dir = config['ply_dir']
    camera_info = config['camera_info']
    rvl_rgbd2ply = RVLRGBD2PLY(sequence_path, rgb_dir, depth_dir, ply_dir, camera_info)
    rvl_rgbd2ply.save_plys()