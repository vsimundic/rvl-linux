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
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import tf
import rospkg
import json
from read_json import read_ao_dict_from_json
import tf2_ros
import moveit_commander


moveit_commander.roscpp_initialize(sys.argv)
node_name = 'save_robot_pose_node'
rospy.init_node(node_name)
pkg_path = '/home/RVLuser/ur5_ws/src/ao_manipulation'

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)

pose = group.get_current_pose().pose

q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
TTB_0 = quaternion_matrix(q)

TTB_0[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])

np.save(os.path.join(pkg_path, 'config', 'TTB_0.npy'), TTB_0)