#!/usr/bin/env python

import numpy as np
import os
import rospkg
import json
from read_json import read_ao_dict_from_json
import tf
from tf.transformations import quaternion_matrix
import geometry_msgs.msg
import rospy


# This is only for setup purposes. It should be just ran once. 
if __name__ == '__main__':

    rospy.init_node('calculate_default_poses_node')
    listener = tf.TransformListener()

    # rospack = rospkg.RosPack()
    # pkg_path = rospack.get_path('ao_manipulation')
    pkg_path = '/home/RVLuser/ur5_ws/src/ao_manipulation'

    # Detected articulated object 
    ao = read_ao_dict_from_json(os.path.join(pkg_path, 'config', 'ao_0.txt'))

    moving_to_static_part_distance = 0.005
    axis_distance = 0.01
    static_side_width = 0.018
    w_door=ao['s'][0]
    h_door=ao['s'][1]

    # Dimensions of the whole cabinet
    w = w_door + 2*(moving_to_static_part_distance + static_side_width)
    h = h_door + 2*(moving_to_static_part_distance + static_side_width)
    static_d = 0.3

    TAC = np.eye(4)
    TAC[:3, :3] = ao['R'].copy()
    TAC[:3, 3] = ao['t'].copy()
    TCA = np.linalg.inv(TAC)
    np.save(os.path.join(pkg_path, 'config', 'TAC_test.npy'), TAC)

    # Align axes of cabinet with world because camera pose TC0 is not known
    TA0 = np.eye(4)
    # theta = np.radians(180)
    # c = np.cos(theta)
    # s = np.sin(theta)
    # Rz = np.array([[c, -s, 0.],
    #                 [s, c, 0.],
    #                 [0., 0., 1.]])
    # TA0[:3, :3] = Rz @ TA0[:3, :3]
    TA0[:3, 3] = np.array([0.6, 0.0, h/2. + 0.005])

    # Camera w.r.t. world 
    TC0 = TA0 @ TCA
    TC0[:3, 3] = np.array([-0.1, 0., TC0[2, 3]])

    # TC0 = np.array([[0, 0, 1, 0],
    #                 [-1, 0, 0, 0],
    #                 [0, -1, 0, 0.7],
    #                 [0, 0, 0, 1]])

    # TA0 = TC0 @ TAC
    # TA0[2, 3] = np.array([h/2. + 0.005])
    # TC0 = TA0 @ TCA
    # TC0[:3, :3] = TCA[:3, :3]



    # TA0[:3, :3] = TAC[:3, :3].copy()

    # Robot base to world
    TB0 = np.eye(4)
    TB0[2, 3] = 0.005

    # Camera w.r.t robot base
    TCB = np.linalg.inv(TB0) @ TC0

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform('tool0', 'camera_link', rospy.Time(0))
            break   
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    # Camera w.r.t robot tool
    TCT = quaternion_matrix(rot)
    TCT[:3, 3] = trans.copy()

    TTB = TCB @ np.linalg.inv(TCT)

    # np.save(os.path.join(pkg_path, 'config', 'TTB_0.npy'), TTB)
    # np.save(os.path.join(pkg_path, 'config', 'TCT_0.npy'), TCT)
    # np.save(os.path.join(pkg_path, 'config', 'TB0_0.npy'), TB0)


    TTB_0 = np.load(os.path.join(pkg_path, 'config', 'TTB_0.npy'))
    TC0 = TB0 @ TTB_0 @ TCT
    TA0 = np.eye(4)
    TA0[:3, 3] = np.array([0, 0.05, h/2.+0.005])
    TAC = np.linalg.inv(TC0) @ TA0
    print(TAC)
