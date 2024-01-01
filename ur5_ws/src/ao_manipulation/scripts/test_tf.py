#!/usr/bin/env python

from std_msgs.msg import Bool
from cv_bridge import CvBridge
import rospy
import os
import numpy as np
import RVLPYDDDetector 
import rosnode
from generate_cabinet_urdf import generate_cabinet_urdf_from_door_panel
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import tf
import rospkg
import json
from read_json import read_ao_dict_from_json
from gazebo_msgs.srv import GetModelState

rospy.init_node('check_tf_node')
listener = tf.TransformListener()

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    try:
        trans, rot = listener.lookupTransform('base_link', 'base_cabinet_link', rospy.Time(0))
        break   
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

TXA = quaternion_matrix(rot)
TXA[:3, 3] = trans.copy()

print(TXA)


try:
    model_coord = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    resp = model_coord('my_cabinet', 'base_cabinet_link')
    print(resp)
except rospy.ServiceException as e:
    rospy.loginfo("Get Model State service call failed:  {0}".format(e))
