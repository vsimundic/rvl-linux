#!/usr/bin/env python

import rospy
import csv
import os
from core.ur5_commander import UR5Commander
from std_msgs.msg import String

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

def main():
    # Initialize the ROS node
    rospy.init_node('test_tsr_node', anonymous=True)

    # Define the file path of the CSV file with joint values
    file_path = os.path.join('/home/RVLuser/ferit_ur5_ws/saved_trajectory.csv')

    # Create an instance of UR5Commander
    ur5_commander = UR5Commander()

    # Load joint values from CSV
    joint_trajectory = read_joint_values_from_csv(file_path)

    # # Send each joint configuration to the UR5 robot
    # for joint_values in joint_trajectory:
    #     ur5_commander.send_joint_values_to_robot(joint_values)
    #     rospy.sleep(1)  # Wait to allow the robot to reach each position

    ur5_commander.send_multiple_joint_space_poses_to_robot(joint_trajectory, 10.)

    rospy.loginfo("Joint trajectory execution completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass