#!/usr/bin/env python

import os
import signal
from subprocess import check_output, CalledProcessError
import rospy
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetModelState
from rospy.exceptions import ROSException
from rosgraph_msgs.msg import Clock
import roslaunch
import csv
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_push_open.cabinet_model import Cabinet
import RVLPYDDManipulator as rvlpy_dd_man
import numpy as np
from numpy.core._exceptions import _ArrayMemoryError
import time
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import subprocess

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

def contact_callback(msg: ContactsState, state_container):
    """
    Callback to update the contact state.

    Args:
        msg (ContactsState): The message containing contact information.
        state_container (dict): A dictionary to hold the contact-free state.
    """
    # global contact_free

    if len(msg.states) > 0:
        # contact_free = False
        state_container['contact_free'] = False

def get_pid(name: str):
    """
    Get the process IDs for a given process name.
    Returns an empty list if no processes are found.
    """
    try:
        return list(map(int, check_output(['pidof', name]).split()))
    except CalledProcessError:
        # No processes found
        return []
    except Exception as e:
        # Handle other unexpected exceptions
        print(f"Error getting PID for {name}: {e}")
        return []

def kill_processes():
    process_names = ['gzserver', 'gzclient', 'rviz', 'move_group', 'robot_state_pub']
    while True:
        any_process_killed = False
        for process_name in process_names:
            pids = get_pid(process_name)
            if pids:
                for pid in pids:
                    try:
                        os.kill(pid, signal.SIGKILL)
                        print(f"Killed {process_name} process with PID {pid}")
                        any_process_killed = True
                    except Exception as e:
                        print(f"Failed to kill {process_name} process with PID {pid}: {e}")
        if not any_process_killed:
            break

def wait_for_clock(timeout=10):
    rospy.loginfo("Waiting for `/clock` topic to publish...")
    try:
        rospy.wait_for_message('/clock', Clock, timeout=timeout)
        rospy.loginfo("`/clock` topic is now publishing.")
    except ROSException:
        rospy.logerr("Timeout exceeded: `/clock` topic is not publishing.")
        return False
    return True

def launch_gazebo(launch_file):
    start = time.time()
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    print("--- %s seconds --- get_or_generate_uuid" % (time.time() - start))

    start = time.time()
    roslaunch.configure_logging(uuid)
    print("--- %s seconds --- configure_logging" % (time.time() - start))

    start = time.time()
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    print("--- %s seconds --- ROSLaunchParent" % (time.time() - start))

    start = time.time()
    rospy.loginfo("Starting Gazebo simulation...")
    launch.start()
    print("--- %s seconds --- start" % (time.time() - start))

    if not wait_for_clock(timeout=15):
        rospy.logerr("Failed to detect `/clock`. Shutting down Gazebo...")
        launch.shutdown()
        exit(1)
    return launch

def start_gazebo_processes(gazebo_launch_file, tf_buffer, T_R_W):
    while True:
        kill_processes()
        gazebo_process = launch_gazebo(gazebo_launch_file)
        print("Gazebo launcher started.")

        reset_tf_buffer(tf_buffer)

        rospy.sleep(1.)
        # Check if Gazebo has started
        if is_gazebo_ready():
            # Check if the robot is in place
            if is_robot_in_place('robot', expected_position=T_R_W[:3,3].tolist()):
                print("Robot is in place.")
            else:
                print("Robot is not in the expected position.")
                stop_gazebo_launcher(gazebo_process)
                continue
            break
        else:
            stop_gazebo_launcher(gazebo_process)
            continue            
    return gazebo_process

def stop_gazebo_launcher(gazebo_process):
    """
    Stop the Gazebo launcher script.
    """

    # Shut down Gazebo
    rospy.loginfo("Shutting down Gazebo...")
    kill_processes()
    gazebo_process.shutdown()
    # rospy.sleep(2)  # Small delay to ensure cleanup before the next iteration

    # try:
    #     os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    #     print("Gazebo launcher terminated.")
    # except Exception as e:
    #     print(f"Failed to terminate Gazebo launcher: {e}")

def is_gazebo_ready(timeout=10):
    """
    Check if Gazebo has started by waiting for the /gazebo/get_model_state service.
    """
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=timeout)
        print("Gazebo is ready.")
        return True
    except rospy.ROSException:
        print("Timeout: Gazebo service not available.")
        return False

def is_robot_in_place(model_name, expected_position, tolerance=0.01):
    """
    Check if the robot is in the expected position in Gazebo.
    
    Parameters:
    ----------
    model_name : str
        The name of the robot model in Gazebo.
    expected_position : list
        The expected [x, y, z] position of the robot.
    tolerance : float
        Allowable deviation in position.
    
    Returns:
    -------
    bool
        True if the robot is within the tolerance range of the expected position.
    """
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state(model_name, 'world')
        actual_position = [
            response.pose.position.x,
            response.pose.position.y,
            response.pose.position.z
        ]
        # Check if the actual position is within the tolerance range
        return all(
            abs(actual - expected) <= tolerance
            for actual, expected in zip(actual_position, expected_position)
        )
    except rospy.ServiceException as e:
        print(f"Failed to get model state: {e}")
        return False


def kill_ros_nodes():
    print("Killing all ROS nodes and processes...")
    # subprocess.run("rosnode kill -a", shell=True)
    subprocess.run("pkill -f gazebo", shell=True)
    subprocess.run("pkill -f move_group", shell=True)
    subprocess.run("pkill -f controller_spawner", shell=True)
    subprocess.run("pkill -f gazebo_controller_spawner", shell=True)
    subprocess.run("pkill -f gazebo_gui", shell=True)
    subprocess.run("pkill -f robot_state_publisher", shell=True)

# def reset_simulation():
#     """
#     Resets the simulation by calling Gazebo reset services and clearing MoveIt! states.
#     """
#     rospy.loginfo("Resetting Gazebo simulation...")

#     # Reset Gazebo world and simulation
#     rospy.wait_for_service('/gazebo/reset_world')
#     rospy.wait_for_service('/gazebo/reset_simulation')
#     rospy.wait_for_service('/move_group/reset_scene')
    
#     try:
#         reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
#         reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
#         reset_scene = rospy.ServiceProxy('/move_group/reset_scene', Empty)

#         reset_world()
#         reset_simulation()
#         reset_scene()
#         rospy.loginfo("Gazebo world and MoveIt! scene reset.")

#     except rospy.ServiceException as e:
#         rospy.logerr(f"Failed to reset simulation: {e}")

#     # # Clear OctoMap (if using perception-based collision avoidance)
#     # rospy.wait_for_service('/clear_octomap')
#     # try:
#     #     clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
#     #     clear_octomap()
#     #     rospy.loginfo("Cleared MoveIt! OctoMap.")
#     # except rospy.ServiceException as e:
#     #     rospy.logwarn(f"Failed to clear OctoMap: {e}")

def reset_tf_buffer(tf_buffer):
    rospy.loginfo("Clearing TF buffer...")
    tf_buffer.clear()  # Ensure the TF buffer is cleared


def attach_two_models(model_name_1, link_name_1, model_name_2, link_name_2, timeout=5.0):
    """
    Attempts to attach two links using the link_attacher service.
    If the operation does not succeed within the timeout, the function exits.

    Args:
        model_name_1 (str): Name of the first model (e.g., 'robot').
        link_name_1 (str): Name of the first link (e.g., 'wrist_3_link').
        model_name_2 (str): Name of the second model (e.g., 'my_cabinet').
        link_name_2 (str): Name of the second link (e.g., 'door_link').
        timeout (float): Maximum time in seconds to attempt attaching before giving up.

    Returns:
        bool: True if attachment succeeded, False otherwise.
    """
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    
    rospy.loginfo('Waiting for /link_attacher_node/attach service...')
    try:
        attach_srv.wait_for_service(timeout=timeout)
    except ROSException:
        rospy.logerr('Service /link_attacher_node/attach not available within timeout.')
        return False

    req = AttachRequest()
    req.model_name_1 = model_name_1
    req.link_name_1 = link_name_1
    req.model_name_2 = model_name_2
    req.link_name_2 = link_name_2

    rospy.loginfo(f'Attempting to attach {model_name_1}:{link_name_1} to {model_name_2}:{link_name_2}...')
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < timeout:
        try:
            attach_srv.call(req)
            rospy.loginfo('Attachment successful.')
            rospy.sleep(1.0)
            return True
        except Exception as e:
            rospy.logwarn(f'Failed to attach: {e}')
            rospy.sleep(0.5)

    rospy.logerr('Failed to attach within the timeout.')
    return False


def detach_two_models(model_name_1, link_name_1, model_name_2, link_name_2, timeout=5.0):
    """
    Attempts to detach two links using the link_attacher service.
    If the operation does not succeed within the timeout, the function exits.

    Args:
        model_name_1 (str): Name of the first model (e.g., 'robot').
        link_name_1 (str): Name of the first link (e.g., 'wrist_3_link').
        model_name_2 (str): Name of the second model (e.g., 'my_cabinet').
        link_name_2 (str): Name of the second link (e.g., 'door_link').
        timeout (float): Maximum time in seconds to attempt detaching before giving up.

    Returns:
        bool: True if detachment succeeded, False otherwise.
    """
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    
    rospy.loginfo('Waiting for /link_attacher_node/detach service...')
    try:
        detach_srv.wait_for_service(timeout=timeout)
    except ROSException:
        rospy.logerr('Service /link_attacher_node/detach not available within timeout.')
        return False

    req = AttachRequest()
    req.model_name_1 = model_name_1
    req.link_name_1 = link_name_1
    req.model_name_2 = model_name_2
    req.link_name_2 = link_name_2

    rospy.loginfo(f'Attempting to detach {model_name_1}:{link_name_1} to {model_name_2}:{link_name_2}...')
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < timeout:
        try:
            detach_srv.call(req)
            rospy.loginfo('Detachment successful.')
            rospy.sleep(1.0)
            return True
        except Exception as e:
            rospy.logwarn(f'Failed to detach: {e}')
            rospy.sleep(0.5)

    rospy.logerr('Failed to detach within the timeout.')
    return False



def rvl_path_planning(rvl_config: str, T_R_W: np.ndarray, q_init: list, num_states: int, return_all_paths: bool, cabinet_model: Cabinet, state: float):

    # Path planning setup
    path_planner = rvlpy_dd_man.PYDDManipulator()
    path_planner.create(rvl_config)
    path_planner.set_robot_pose(T_R_W)
    path_planner.set_door_model_params(
                                    cabinet_model.d_door,
                                    cabinet_model.w_door,
                                    cabinet_model.h_door,
                                    0.0, # rx
                                    -(cabinet_model.w_door*0.5 - cabinet_model.axis_distance), # ry
                                    cabinet_model.axis_pos, # opening direction
                                    cabinet_model.static_side_width,
                                    cabinet_model.moving_to_static_part_distance)
    path_planner.set_door_pose(cabinet_model.T_A_S)
    path_planner.set_environment_state(state)


    all_feasible_paths = None
    all_feasible_paths_q = None
    # Plan
    try:
        if return_all_paths:
            T_G_0_array, q, all_feasible_paths, all_feasible_paths_q = path_planner.path2(np.array(q_init), -90.0, num_states, return_all_paths)
        else:
            T_G_0_array, q = path_planner.path2(np.array(q_init), -90.0, num_states, return_all_paths)
    except (_ArrayMemoryError, ValueError) as e:
        T_G_0_array = np.zeros((1,4,4))

    del path_planner

    return T_G_0_array, q, all_feasible_paths, all_feasible_paths_q

def chebyshev_distance(q1, q2):
    q1 = np.array(q1)
    q2 = np.array(q2)
    error = np.abs(q1 - q2)
    max_error = np.max(error)
    return max_error


def fix_joint_angles(joint_angles, threshold_factor=0.98):
    """
    Adjusts joint angles to ensure smooth transitions without unnecessary full-circle rotations.
    
    Parameters:
        joint_angles (numpy.ndarray): A (N, M) array where N is the number of points and M is the number of joints.
        threshold_factor (float): The fraction of π to use as a threshold (e.g., 0.98 means 98% of π).
        
    Returns:
        numpy.ndarray: Adjusted joint angles.
    """
    adjusted_angles = joint_angles.copy()
    threshold = threshold_factor * np.pi  # Compute dynamic threshold based on π

    for j in range(joint_angles.shape[1]):  # Loop over each joint
        for i in range(2, joint_angles.shape[0]):  # Loop over points
            diff = adjusted_angles[i, j] - adjusted_angles[i - 1, j]
            
            # If the difference is greater than threshold (but not necessarily full 2π), correct it
            if diff > threshold:
                adjusted_angles[i, j] -= 2 * np.pi
            elif diff < -threshold:
                adjusted_angles[i, j] += 2 * np.pi

    return adjusted_angles