import rospy
from typing import Union
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesResponse
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import Pose

def get_joint_info(joint_name: str) -> Union[GetJointPropertiesResponse, None]:
    """
    Retrieve properties of a specific joint from Gazebo.

    :param joint_name: Name of the joint in Gazebo
    :return: GetJointPropertiesResponse with joint properties (position, rate, damping) 
            or None if the request fails
    """
    rospy.wait_for_service('/gazebo/get_joint_properties')
    try:
        get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        response = get_joint_properties(joint_name)

        if response.success:
            rospy.loginfo(f"Joint: {joint_name}")
            rospy.loginfo(f"Position: {response.position}")
            rospy.loginfo(f"Rate: {response.rate}")
            rospy.loginfo(f"Damping: {response.damping}")
            return response
        else:
            rospy.logwarn(f"Failed to get joint properties for: {joint_name}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def get_link_pose(model_name: str, link_name: str) -> Union[Pose, None]:
    """
    Retrieve the pose of a specific link in a given model from Gazebo.

    :param model_name: Name of the model in Gazebo
    :param link_name: Name of the link within the model
    :return: Pose of the link in the world frame, or None if unsuccessful
    """
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        # Create a service proxy for /gazebo/get_link_state
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        # Full link name in Gazebo is usually "model::link"
        full_link_name = f"{model_name}::{link_name}"
        
        # Request the pose of the link
        response = get_link_state(full_link_name, "world")  # Reference frame is "world"

        if response.success:
            rospy.loginfo(f"Pose of {full_link_name}: {response.link_state.pose}")
            return response.link_state.pose
        else:
            rospy.logwarn(f"Failed to get pose for link: {full_link_name}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None
