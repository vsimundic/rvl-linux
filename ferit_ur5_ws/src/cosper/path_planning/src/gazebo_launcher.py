#!/usr/bin/env python

import roslaunch
import rospy
from rospy.exceptions import ROSException
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

def wait_for_clock(timeout=10):
    """
    Wait for the /clock topic to publish, indicating Gazebo is running.
    """
    rospy.loginfo("Waiting for `/clock` topic to publish...")
    try:
        rospy.wait_for_message('/clock', Clock, timeout=timeout)
        rospy.loginfo("`/clock` topic is now publishing.")
    except ROSException:
        rospy.logerr("Timeout exceeded: `/clock` topic is not publishing.")
        return False
    return True

def reset_tf_buffer(tf_buffer):
    rospy.loginfo("Clearing TF buffer...")
    tf_buffer.clear()  # Ensure the TF buffer is cleared


if __name__ == "__main__":
    rospy.init_node("gazebo_launcher", anonymous=True)

    # TF buffer setup
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Path to your Gazebo launch file
    gazebo_launch_file = "/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [gazebo_launch_file])

    rospy.loginfo("Starting Gazebo simulation...")
    launch.start()

    # Wait for `/clock` to publish to ensure Gazebo is running
    if not wait_for_clock(timeout=15):
        rospy.logerr("Failed to detect `/clock`. Shutting down Gazebo...")
        launch.shutdown()
        exit(1)  # Exit with failure code


    reset_tf_buffer(tf_buffer)


    try:
        rospy.spin()  # Keep the script running until terminated
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting down Gazebo simulation...")
        launch.shutdown()
        rospy.sleep(2)  