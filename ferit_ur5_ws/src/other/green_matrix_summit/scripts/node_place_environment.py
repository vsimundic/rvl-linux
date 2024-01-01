#!/usr/bin/python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("place_environment_node", anonymous=True)
    ground_name = 'ground'
    scene = moveit_commander.PlanningSceneInterface()

    is_known = ground_name in scene.get_known_object_names()

    if not is_known:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.01
        box_name = ground_name
        scene.add_box(box_name, box_pose, size=(1, 1, 0.01))