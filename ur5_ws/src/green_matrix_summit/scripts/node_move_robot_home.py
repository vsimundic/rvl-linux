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

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_home_node', anonymous=True)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('ao_manipulation')

    imgs_pub = rospy.Publisher('/save_images_start', Bool, queue_size=1)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = 'arm'
    group = moveit_commander.MoveGroupCommander(group_name)

    TTB = np.load(os.path.join(pkg_path, 'config', 'TTB_0.npy'))
    
    q = quaternion_from_matrix(TTB) # xyzw

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    pose_goal.position.x = TTB[0, 3]
    pose_goal.position.y = TTB[1, 3]
    pose_goal.position.z = TTB[2, 3]
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(5)
    msg = Bool()
    msg.data = True
    imgs_pub.publish(msg)
    