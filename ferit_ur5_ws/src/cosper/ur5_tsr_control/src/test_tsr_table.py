#!/usr/bin/env python

import rospy
import csv
import os
from core.ur5_commander import UR5Commander
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from core.transforms  import matrix_to_pose
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
import numpy as np
from gazebo_msgs.msg import ModelState
import tf
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty


# Function to read joint values from a CSV file
def read_joint_values_from_csv(file_path) -> list:
    joint_values = []
    with open(file_path, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file)
        headers = next(csv_reader)
        for row in csv_reader:
            # Convert each row to a list of floats
            joint_values.append([float(value) for value in row[-6:]])
    return joint_values

def spawn_model(model_name, model_path, T=np.ndarray, frame_id="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Read the URDF file
    with open(model_path, 'r') as file:
        model_xml = file.read()

    # Define the pose for the model
    pose = matrix_to_pose(T)

    try:
        # Call the spawn model service
        spawn_model_service(model_name=model_name, model_xml=model_xml, robot_namespace='', initial_pose=pose, reference_frame=frame_id)
        rospy.loginfo(f"Successfully spawned {model_name} at ({pose.position.x}, {pose.position.y}, {pose.position.z}) with reference frame '{frame_id}'")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model_name}: {e}")

def delete_gazebo_model(model_name):
    """Delete a model in Gazebo."""
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model(model_name)
        rospy.loginfo(f"Model '{model_name}' deleted.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Delete Model service call failed: {e}")

def attach_can_to_tool0():
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Set the can's initial pose relative to tool0
    state_msg = ModelState()
    state_msg.model_name = 'can'
    state_msg.reference_frame = 'tool0'  # Attach the can to tool0 frame
    state_msg.pose.position.x = 0.0
    state_msg.pose.position.y = 0.7
    state_msg.pose.position.z = 0.0
    quat = quaternion_from_euler(0, 0, 0)
    state_msg.pose.orientation.x = quat[0]
    state_msg.pose.orientation.y = quat[1]
    state_msg.pose.orientation.z = quat[2]
    state_msg.pose.orientation.w = quat[3]

    try:
        # Apply the state to "attach" the can to tool0
        set_model_state(state_msg)
        rospy.loginfo("Can is now 'attached' to tool0.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to attach can to tool0: {e}")

def update_can_position(tf_listener):
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rate = rospy.Rate(10)  # Update rate in Hz

    while not rospy.is_shutdown():
        try:
            # Get the transform from 'tool0' to 'world'
            (trans, rot) = tf_listener.lookupTransform('/world', '/tool0', rospy.Time(0))

            # Set the can's position to match the tool0 frame in world coordinates
            state_msg = ModelState()
            state_msg.model_name = 'can'
            state_msg.reference_frame = 'world'  # Moving in world frame with tool0 position
            state_msg.pose.position.x = trans[0]
            state_msg.pose.position.y = trans[1] + 0.3
            state_msg.pose.position.z = trans[2]
            state_msg.pose.orientation.x = rot[0]
            state_msg.pose.orientation.y = rot[1]
            state_msg.pose.orientation.z = rot[2]
            state_msg.pose.orientation.w = rot[3]

            # Apply the state
            set_model_state(state_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get transform from tool0 to world.")
        
        rate.sleep()


def main():
    # Initialize the ROS node
    rospy.init_node('test_tsr_node', anonymous=True)

    # Spawn table model near the robot
    table_urdf_path = "/home/RVLuser/ferit_ur5_ws/src/cosper/ur5_tsr_control/table.urdf"
    T_t_W = np.eye(4)
    T_t_W[:3,  3] = np.array([0.5, 0.5, 0.201])
    # delete_gazebo_model('table')
    # spawn_model("table", table_urdf_path, T_t_W)
    # Wait a moment to ensure the can is spawned
    # rospy.sleep(2)

    # Spawn can model in robot's hand
    can_urdf_path = "/home/RVLuser/ferit_ur5_ws/src/cosper/ur5_tsr_control/can.urdf"
    T_c_P = np.eye(4)
    T_c_P[:3,  3] = np.array([0.0, 0.7, 0.2])
    # delete_gazebo_model('can')
    # spawn_model("can", can_urdf_path, T_c_P, frame_id='world')
    # rospy.sleep(2) # Wait a moment to ensure the can is spawned

    print("test")
    # Define the file path of the CSV file with joint values
    file_path = os.path.join('/home/RVLuser/ferit_ur5_ws/saved_trajectory_table.csv')
    # Attach the can to tool0 so it moves with the robot's end-effector

    # Create an instance of UR5Commander
    ur5_commander = UR5Commander()

    # Load joint values from CSV
    joint_trajectory = read_joint_values_from_csv(file_path)

    # # Send each joint configuration to the UR5 robot
    # for joint_values in joint_trajectory:
    #     ur5_commander.send_joint_values_to_robot(joint_values)
    #     rospy.sleep(1)  # Wait to allow the robot to reach each position
    print("test2")
    ur5_commander.send_multiple_joint_space_poses_to_robot(joint_trajectory, 15.)

    rospy.loginfo("Joint trajectory execution completed.")




    # # Create an instance of UR5Commander
    # ur5_commander = UR5Commander()

    # # Load joint values from CSV
    # joint_trajectory = read_joint_values_from_csv(file_path)

    # # # Send each joint configuration to the UR5 robot
    # # for joint_values in joint_trajectory:
    # #     ur5_commander.send_joint_values_to_robot(joint_values)
    # #     rospy.sleep(1)  # Wait to allow the robot to reach each position

    # ur5_commander.send_multiple_joint_space_poses_to_robot(joint_trajectory, 10.)

    # rospy.loginfo("Joint trajectory execution completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass