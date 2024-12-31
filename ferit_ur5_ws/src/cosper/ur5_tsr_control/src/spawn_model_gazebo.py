#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
import os
from rospy import get_param
import numpy as np
from core.transforms import matrix_to_pose
def delete_gazebo_model(model_name):
    """Delete a model in Gazebo."""
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model(model_name)
        rospy.loginfo(f"Model '{model_name}' deleted.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Delete Model service call failed: {e}")

def spawn_gazebo_model(model_name, model_path, model_pose):
    """Spawn a model in Gazebo."""
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        
        # Read the URDF file
        with open(model_path, "r") as model_file:
            model_xml = model_file.read()
        
        # Spawn the model
        resp = spawn_model(model_name, model_xml, "", model_pose, "world")
        rospy.loginfo(f"Model '{model_name}' spawned.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn Model service call failed: {e}")

def spawn_model_wrt_frame(model_name, urdf_path, frame_id="world", T=np.eye(4)):
    rospy.init_node("spawn_model_node", anonymous=True)

    # Wait for the spawn model service to be available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Load the URDF file
    # Read the URDF file
    with open(urdf_path, "r") as model_file:
        model_xml = model_file.read()

    # Create the pose relative to the frame (world, or any fixed frame)
    pose = matrix_to_pose(T)

    # Spawn the model in Gazebo
    try:
        resp = spawn_model_service(model_name=model_name, model_xml=model_xml, robot_namespace='', initial_pose=pose, reference_frame=frame_id)
        rospy.loginfo(f"Successfully spawned model {model_name} in Gazebo")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")




if __name__ == "__main__":
    rospy.init_node("spawn_delete_object_node", anonymous=True)
    


    ### TABLE
    # Define model name and paths
    model_name = "table"
    model_path = "/home/RVLuser/ferit_ur5_ws/src/cosper/ur5_tsr_control/table.urdf"  # Update this path

    # Set the initial position of the object
    model_pose = Pose()
    model_pose.position.x = 1.0
    model_pose.position.y = 1.0
    model_pose.position.z = 0.35  # Height based on tabletop origin on surface

    # Delete the model if it already exists
    delete_gazebo_model(model_name)

    # Spawn the model
    spawn_gazebo_model(model_name, model_path, model_pose)


    ### CAN
    can_model_name = 'can'
    T_C_6 = np.eye(4)
    T_C_6[:3, 3] = np.array([0., 0., 0.3])
    T_C_6[3, 3] = np.array([0., 0., 0.3])

    rospy.spin()