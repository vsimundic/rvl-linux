#!/usr/bin/python

import rospy
import moveit_commander
import numpy as np
import os

rospy.init_node('moveit_planning_dd_man_node')

def main():
    
    # Load the gripper pose filename 
    gripper_final_pose_path = rospy.get_param('~gripper_pose_path')
    
    
    urdf_rootpath = os.path.dirname(os.path.abspath(urdf_filepath))



    pass

if __name__ == '__main__':
    main()