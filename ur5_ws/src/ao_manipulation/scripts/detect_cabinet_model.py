#!/usr/bin/env python

import rosbag
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32, String, Bool
from cv_bridge import CvBridge
import cv2
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
import os
import numpy as np
import RVLPYRGBD2PLY
import RVLPYDDDetector 
import rosnode

node_name = ''
cfg_path = ''
rgb_save_path = ''
depth_save_path = ''
ply_save_path = ''

def callback(b_build_model_msg):
    global node_name, cfg_path, rgb_save_path, depth_save_path, ply_save_path
    if b_build_model_msg:
        detect_model(cfg_path, rgb_save_path, depth_save_path, ply_save_path)
        rosnode.kill_nodes([node_name])

def detect_model(cfg_path, rgb_save_path, depth_save_path, ply_save_path):
    detector = RVLPYDDDetector.PYDDDetector()
    detector.create(cfg_path)

    for filename in os.listdir(ply_save_path):
        ply_path = os.path.join(ply_save_path, filename)
        detector.add_mesh(ply_path)

    for filename in os.listdir(rgb_save_path):
        rgb_path = os.path.join(rgb_save_path, filename)
        detector.add_rgb(rgb_path)

    ao = detector.detect()
    print(ao)


if __name__ == '__main__':

    node_name = 'detect_cabinet_model_node'

    cfg_path = rospy.get_param('/rvl_cfg_path')
    rgb_save_path = rospy.get_param('/rgb_save_path')
    depth_save_path = rospy.get_param('/depth_save_path')
    ply_save_path = rospy.get_param('/ply_save_path')

    rospy.init_node(node_name)
    build_model_sub = rospy.Subscriber('/build_model', Bool, callback)
    rospy.spin()

