#!/usr/bin/env python3

import rospy


if __name__=='__main__':
    rospy.init_node("test_node")#za inicijalizaciju cvora

    rospy.loginfo("Test node has been started.")

    rate=rospy.Rate(10)
    while not rospy.is_shutdown():#dok čvor nije ugasen ctrl+c
        rospy.loginfo("Hello")
        rate.sleep()#loop je na frekvenciji 10 Hz
    