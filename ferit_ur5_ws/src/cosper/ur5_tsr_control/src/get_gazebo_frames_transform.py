#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, GetLinkStateResponse

def get_link_transform(robot_link, can_link):
    rospy.init_node('gazebo_link_transform_printer', anonymous=True)

    # Wait for the service to be available
    rospy.wait_for_service('/gazebo/get_link_state')
    
    try:
        # Create the service proxy
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        # Set up the request for the links of interest
        request = GetLinkStateRequest()
        request.link_name = can_link
        request.reference_frame = robot_link  # Set the reference frame to the robot link

        # Query the transformation in a loop
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            response = get_link_state(request)
            
            # Check if the response was successful
            if response.success:
                position = response.link_state.pose.position
                orientation = response.link_state.pose.orientation

                print(f"Translation from {robot_link} to {can_link}:")
                print(f"  x: {position.x}")
                print(f"  y: {position.y}")
                print(f"  z: {position.z}")

                print(f"Rotation from {robot_link} to {can_link} (quaternion):")
                print(f"  x: {orientation.x}")
                print(f"  y: {orientation.y}")
                print(f"  z: {orientation.z}")
                print(f"  w: {orientation.w}")

            else:
                rospy.logwarn(f"Failed to get transform from {robot_link} to {can_link}")

            rate.sleep()

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        # Replace 'ur5::gripper_link' and 'can::can_link' with your Gazebo model links
        get_link_transform('world', 'table::table_top')
    except rospy.ROSInterruptException:
        pass