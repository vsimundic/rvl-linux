#!/usr/bin/python

import rospy
import os
from core.util import read_config, read_csv_DataFrame
from core.ur5_commander import UR5Commander
from gazebo_push_open.cabinet_model import Cabinet
import numpy as np
from core.transforms import rot_z, pose_to_matrix
import RVLPYDDManipulator as rvlpy_dd_man
import roslaunch
from gazebo_msgs.msg import ContactsState
from subprocess import check_output
import signal
import csv
from numpy.core._exceptions import _ArrayMemoryError
from gazebo_msgs.msg import ContactsState
from PIL import ImageGrab
from rospkg import RosPack
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from core.gazebo import get_joint_info, get_link_pose

rospy.init_node('test_ik')

successful_indices = [19, 22, 38, 43, 51, 59, 67, 74, 81, 83, 87, 89, 93, 118, 122, 149, 158, 159, 174, 187, 192, 214, 215, 257, 260, 264, 279, 313, 348, 392, 396, 398, 411, 426, 436, 471, 494, 515, 525, 546, 548, 550, 555, 558, 570, 579, 583, 592, 611, 655, 677, 683, 711, 724, 729, 763, 770, 775, 779, 787, 788, 797, 806, 813, 826, 829, 866, 875, 890, 905, 908, 928, 939, 940, 941, 970, 974, 979, 986, 988, 993, 999]

idx = successful_indices[0]

pathT0_w = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/ikfast_successful_matrices/cabinet_%d_T0_w2.npy' % idx
pathTw_e = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/ikfast_successful_matrices/cabinet_%d_Tw_e2.npy' % idx

T0_w = np.load(pathT0_w)
Tw_e = np.load(pathTw_e)

T0_e = T0_w @ Tw_e

robot = UR5Commander()
rospy.sleep(1)

q_init = [0.0, float(np.deg2rad(-90.)), 0.0, float(np.deg2rad(-90.)), 0.0, 0.0]
q = robot.get_inverse_kin(q_init, T0_e)

if q is None:
    print('No IK solution found.')
else:
    print('Found IK solution: ', q)