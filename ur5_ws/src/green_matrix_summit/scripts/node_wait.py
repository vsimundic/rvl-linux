#!/usr/bin/env python

import numpy as np
import os
import sys
import rospkg
from tf.transformations import quaternion_from_matrix
import geometry_msgs.msg
import rospy
import moveit_commander
from std_msgs.msg import Bool


rospy.init_node('node_wait', anonymous=True)
imgs_pub = rospy.Publisher('/save_images_start', Bool, queue_size=1)
rospy.sleep(8)
msg = Bool()
msg.data = True
imgs_pub.publish(msg)