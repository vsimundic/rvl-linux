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
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix
import rospkg
import json

node_name = ''
cfg_path = ''
rgb_save_path = ''
depth_save_path = ''
ply_save_path = ''
package_path = ''
b_load_model = False

def callback(b_build_model_msg):
    global node_name, cfg_path, rgb_save_path, depth_save_path, ply_save_path, package_path, b_load_model
    if b_build_model_msg:
        ao = {}
        if not b_load_model:
            ao = detect_model(cfg_path, rgb_save_path, depth_save_path, ply_save_path)
            print(ao)
        else:
            with open(os.path.join(package_path, 'config', 'ao.txt')) as f:
                data = f.read()
                ao_js = json.loads(data)

                for key in ao_js:
                    if isinstance(ao_js[key], list):
                        ao[key] = np.array(ao_js[key])

        TAC = np.eye(4)
        TAC[:3, :3] = ao['R'].copy()
        TAC[:3, 3] = ao['t']

        model_name = 'my_cabient'


        # Delete model if exists
        delete_model(model_name)

        spawn_model(w_door=ao['s'][0], h_door=ao['s'][1], TAC=TAC, model_name=model_name)
        move_model_joint(model_name, 'joint_0', np.radians(-20))
        rosnode.kill_nodes([node_name])

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        _ = spawn_urdf_model_proxy(model_name)
        rospy.loginfo('URDF deleted successfully')
    except rospy.ServiceException as e:
        rospy.logerr('Failed to delete model: %s' % e)

    

def spawn_model(w_door, h_door, TAC, model_name):
    model_xml = generate_cabinet_urdf_from_door_panel(w_door=w_door, h_door=h_door, d_door=0.018)

    init_pose = Pose()
    init_pose.position.x = TAC[0, 2]
    init_pose.position.y = TAC[1, 2]
    init_pose.position.z = TAC[2, 2] + 0.5

    q = quaternion_from_matrix(TAC)

    init_pose.orientation.x = q[0]
    init_pose.orientation.y = q[1]
    init_pose.orientation.z = q[2]
    init_pose.orientation.w = q[3]

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        _ = spawn_urdf_model_proxy(model_name, model_xml, '/', init_pose, 'base_link')
        rospy.loginfo('URDF model spawned successfully')
    except rospy.ServiceException as e:
        rospy.logerr('Failed to spawn URDF model: %s' % e)


def move_model_joint(model_name, joint_name, joint_value):
    rospy.wait_for_service('/gazebo/set_model_configuration')

    try:
        set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        req = SetModelConfigurationRequest()
        req.model_name = model_name
        req.urdf_param_name = "robot_description"  # Assuming your URDF is loaded with the parameter name "robot_description"

        req.joint_names = [joint_name]
        req.joint_positions = [joint_value]

        _ = set_model_configuration(req)
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
    print(ao)
    

    # Prepare for saving
    ao_js = {}
    for key in ao:
        if isinstance(ao[key], np.ndarray):
            ao_js[key] = ao[key].tolist()
        else:
            ao_js[key] = ao[key]
    
    print(ao_js)

    with open(os.path.join(package_path, 'config', 'ao.txt'), 'w') as f:
        f.write(json.dumps(ao_js))
    
    return ao


if __name__ == '__main__':

    node_name = 'detect_cabinet_model_node'
    rospy.init_node(node_name)

    cfg_path = rospy.get_param('/rvl_cfg_path')
    rgb_save_path = rospy.get_param('/rgb_save_path')
    depth_save_path = rospy.get_param('/depth_save_path')
    ply_save_path = rospy.get_param('/ply_save_path')
    b_load_model = rospy.get_param('~load_model')


    rospack = rospkg.RosPack()
    package_path = rospack.get_path('ao_manipulation')

    build_model_sub = rospy.Subscriber('/build_model', Bool, callback)
    rospy.spin()

