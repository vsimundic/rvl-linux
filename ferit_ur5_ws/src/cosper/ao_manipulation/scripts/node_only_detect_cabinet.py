#!/usr/bin/env python

from std_msgs.msg import Bool
from cv_bridge import CvBridge
import rospy
import os, sys
import numpy as np
import RVLPYDDDetector 
import rosnode
from generate_cabinet_urdf import generate_cabinet_urdf_from_door_panel
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel
from geometry_msgs.msg import Pose, TransformStamped
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix
import tf
import rospkg
import json
from read_json import read_ao_dict_from_json
import tf2_ros
import moveit_commander

node_name = ''
cfg_path = ''
rgb_save_path = ''
depth_save_path = ''
ply_save_path = ''
package_path = ''
b_load_model = False
listener = ''
broadcaster = ''
pus_operation_pub = ''
TCT = ''
TTB = ''
TB0 = ''
scene = ''


def callback(b_build_model_msg):
    global node_name, cfg_path, rgb_save_path, depth_save_path, ply_save_path, package_path, b_load_model

    b_load_model = rospy.get_param('/load_model')

    if b_build_model_msg:
        ao = {}
        if not b_load_model:
            ao = detect_model(cfg_path, rgb_save_path, depth_save_path, ply_save_path)
            print(ao)


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
    return ao


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    node_name = 'detect_cabinet_model_node'
    rospy.init_node(node_name)

    scene = moveit_commander.PlanningSceneInterface()

    cfg_path = rospy.get_param('/rvl_cfg_path')
    rgb_save_path = rospy.get_param('/rgb_save_path')
    depth_save_path = rospy.get_param('/depth_save_path')
    ply_save_path = rospy.get_param('/ply_save_path')
    b_load_model = rospy.get_param('/load_model')

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('ao_manipulation')

    build_model_sub = rospy.Subscriber('/build_model2', Bool, callback)
    rospy.spin()

