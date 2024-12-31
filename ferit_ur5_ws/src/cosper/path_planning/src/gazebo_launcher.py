#!/usr/bin/env python

import roslaunch
import rospy

if __name__ == "__main__":
    rospy.init_node("gazebo_launcher", anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"]
    )
    rospy.loginfo("Starting Gazebo simulation...")
    launch.start()
    try:
        rospy.spin()  # Keep the script running until terminated
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting down Gazebo simulation...")
        launch.shutdown()