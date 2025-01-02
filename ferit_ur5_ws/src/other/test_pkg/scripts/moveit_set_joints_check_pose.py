#!/usr/bin/env python

import rospy
import numpy as np
from core.ur5_commander import UR5Commander
np.set_printoptions(suppress=True)

def main():
    rospy.init_node('moveit_set_joints_check_pose')

    # Initialize UR5Commander
    ur5 = UR5Commander()
    
    # Remove all objects from the scene
    ur5.clear_planning_scene()
    
    # Set joint values
    joint_values = np.array([3.14123, -1.45858, 1.95458, 2.64572, 0.000845728, 3.14147])
    joint_values[0] += np.pi
    joint_values[5] += np.pi
    joint_values[joint_values>np.pi]-=(2.0*np.pi)     
    joint_values[joint_values<-np.pi]+=(2.0*np.pi)
    print(joint_values)

    ur5.send_named_pose('up')

    ur5.send_joint_values_to_robot(joint_values.tolist())

    # Check pose
    pose = ur5.get_current_tool_pose()

    print('Pose:\n {}'.format(pose))

if __name__ == '__main__':
    main()