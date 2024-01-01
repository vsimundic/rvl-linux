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
from gazebo_ros import gazebo_interface
from generate_cabinet_urdf import generate_cabinet_urdf_from_door_panel
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix

node_name = ''
cfg_path = ''
rgb_save_path = ''
depth_save_path = ''
ply_save_path = ''

def callback(b_build_model_msg):
    global node_name, cfg_path, rgb_save_path, depth_save_path, ply_save_path
    if b_build_model_msg or True:
        ao = detect_model(cfg_path, rgb_save_path, depth_save_path, ply_save_path)
        print(ao)

        TAC = np.eye(4)
        TAC[:3, :3] = np.array(ao['R']).reshape((3,3))
        TAC[:3, 3] = np.array(ao['t'])

        model_name = 'my_cabient'

        spawn_model(w_door=ao['s'][0], h_door=ao['s'][1], TAC=TAC, model_name=model_name)
        move_model_joint(model_name, 'joint_0', np.radians(15))
        rosnode.kill_nodes([node_name])

def spawn_model(w_door, h_door, TAC, model_name):
    model_xml = generate_cabinet_urdf_from_door_panel(w_door=w_door, h_door=h_door, d_door=0.018)

    init_pose = Pose()
    init_pose.position.x = TAC[0, 2]
    init_pose.position.y = TAC[1, 2]
    init_pose.position.z = TAC[2, 2]

    q = quaternion_from_matrix(TAC)

    init_pose.orientation.x = q[0]
    init_pose.orientation.y = q[1]
    init_pose.orientation.z = q[2]
    init_pose.orientation.w = q[3]

    spawn_model_msg = SpawnModel()
    spawn_model_msg.model_name = model_name
    spawn_model_msg.model_xml = model_xml
    spawn_model_msg.robot_namespace = '/'
    spawn_model_msg.initial_pose = init_pose

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        response = spawn_urdf_model_proxy(spawn_model_msg)
        rospy.loginfo('URDF model spawned successfully')
    except rospy.ServiceException as e:
        rospy.logerr('Failed to spawn URDF model: %s' % e)

    move_model_joint('my_cabinet', 'joint_0', np.radians(20))

def move_model_joint(model_name, joint_name, joint_value):
    rospy.wait_for_service('/gazebo/set_model_configuration')

    try:
        set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        req = SetModelConfigurationRequest()
        req.model_name = model_name
        req.urdf_param_name = "robot_description"  # Assuming your URDF is loaded with the parameter name "robot_description"

        joint_position = {joint_name: joint_value}
        req.joint_names = [joint_name]
        req.joint_positions = [joint_value]

        response = set_model_configuration(req)
        rospy.loginfo(f"Joint {joint_name} set to {joint_value} for model {model_name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to set joint configuration: {e}")


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

    node_name = 'detect_cabinet_model_node'

    cfg_path = rospy.get_param('/rvl_cfg_path')
    rgb_save_path = rospy.get_param('/rgb_save_path')
    depth_save_path = rospy.get_param('/depth_save_path')
    ply_save_path = rospy.get_param('/ply_save_path')

    rospy.init_node(node_name)
    build_model_sub = rospy.Subscriber('/build_model', Bool, callback)
    rospy.spin()

