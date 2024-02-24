#!/usr/bin/python

import rospy
from core.image_process import ImageProcess
from core.ur5_commander import UR5Commander

if __name__ == '__main__':
    rospy.init_node('node_capture_images')

    save_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-depth_rgb_check-20240217'
    rgb_topic = '/camera/rgb/image_raw'
    depth_topic = '/camera/depth_registered/image_raw'
    camera_info_topic = '/camera/rgb/camera_info'
    camera_fps = 30
    wanted_fps = 5
    depth_encoding = '32FC1'

    # Robot handler
    robot = UR5Commander()

    ip = ImageProcess(save_dir, rgb_topic, depth_topic, camera_info_topic, camera_fps, depth_encoding, robot)

    rgb_save_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-depth_rgb_check-20240217/rgb'
    depth_save_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-depth_rgb_check-20240217/depth'
    ip.save_rgb_depth(wanted_fps)

    rospy.spin()