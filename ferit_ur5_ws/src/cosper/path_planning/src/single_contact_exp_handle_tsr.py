#!/usr/bin/python

import rospy
import os
from core.util import read_config, read_csv_DataFrame
from core.ur5_commander import UR5Commander
from gazebo_push_open.cabinet_model import Cabinet
import numpy as np
from core.transforms import rot_z, pose_to_matrix
import RVLPYDDManipulator as rvlpy_dd_man
import roslaunch
from gazebo_msgs.msg import ContactsState
from subprocess import check_output
import signal
import csv
from numpy.core._exceptions import _ArrayMemoryError
from gazebo_msgs.msg import ContactsState
from PIL import ImageGrab
from rospkg import RosPack
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from core.gazebo import get_joint_info, get_link_pose
from gazebo_msgs.srv import GetModelState
import subprocess
from subprocess import check_output, CalledProcessError

np.random.seed(69)

contact_free = True

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

def contact_callback(msg: ContactsState):
    global contact_free

    if len(msg.states) > 0:
        contact_free = False

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


def start_gazebo_launcher():
    """
    Start the Gazebo launcher script.
    """
    process = subprocess.Popen(
        ["python", "/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/src/gazebo_launcher.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid  # Allows killing the process group later
    )
    return process

def stop_gazebo_launcher(process):
    """
    Stop the Gazebo launcher script.
    """
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        print("Gazebo launcher terminated.")
    except Exception as e:
        print(f"Failed to terminate Gazebo launcher: {e}")

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

if __name__ == '__main__':
    rospy.init_node('single_contact_exp_tsr')
    
    rp = RosPack()
    pkg_path = rp.get_path('path_planning')

    is_saving_results = True

    # Config
    cfg_path = os.path.join(pkg_path, 'config/config_tsr_simulations_axis_left.yaml')
    config = read_config(cfg_path)

    # Save/load path for results
    csv_path = os.path.join(pkg_path,'results_simulation_tsr_single_contact.csv')

    # TSR trajectories path
    tsr_trajectories_path = config['tsr_trajectories_path']

    # Load door configurations
    door_configs_path = os.path.join(pkg_path, 'door_configurations_axis_left.npy')
    doors = np.load(door_configs_path)
    
    is_saving_images = False
    save_screenshot_path = os.path.join(pkg_path, 'single_contact_screenshots')

    # If False, the data loads and the experiment starts where it stopped
    start_from_beginning = True
    start_i = 0
    if start_from_beginning:
        if is_saving_results:
            with open(csv_path, 'w') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(['path_found', 'traj_success', 'contact_free', 'door_opened', 'door_width', 'door_height', 'x', 'y', 'z', 'rot_z', 'state_angle', 'axis_pos'])
    else:
        data = read_csv_DataFrame(csv_path)
        start_i = data.shape[0]
        # doors = doors[rows:, :]

    
    T_R_W = np.eye(4)
    T_R_W[2, 3] = 0.005

    successful_indices = [19, 22, 38, 43, 51, 59, 67, 74, 81, 83, 87, 89, 93, 118, 122, 149, 158, 159, 174, 187, 192, 214, 215, 257, 260, 264, 279, 313, 348, 392, 396, 398, 411, 426, 436, 471, 494, 515, 525, 546, 548, 550, 555, 558, 570, 579, 583, 592, 611, 655, 677, 683, 711, 724, 729, 763, 770, 775, 779, 787, 788, 797, 806, 813, 826, 829, 866, 875, 890, 905, 908, 928, 939, 940, 941, 970, 974, 979, 986, 988, 993, 999]

    i = start_i
    # for i in range(start_i, doors.shape[0]):
    while i < 1000:
        try:
            path_found = False
            trajectory_successful = False
            door_opened = False
            contact_free = True
            final_success = False

            door = doors[successful_indices[i], :]
            width = door[0]
            height = door[1]
            position = door[2:5]
            rot_z_deg = door[5]
            state_angle = door[6]
            axis_pos = door[7]

            traj_filename = os.path.join(tsr_trajectories_path, 'traj_%d.csv' %i)
            q_traj = read_csv_DataFrame(traj_filename)
            if q_traj is None or q_traj.empty:
                with open(csv_path, 'a') as f:
                    writer = csv.writer(f, delimiter=',')
                    writer.writerow([path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])
                    i+=1
                    continue

            # convert to list of lists
            q_traj = q_traj.to_numpy().tolist()

            T_A_S = np.eye(4)
            T_A_S[:3, 3] = np.array(position)
            T_A_S[2, 3] += T_R_W[2, 3]
            Tz = np.eye(4)
            Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
            T_A_S = T_A_S @ Tz
            
            # Create a cabinet object
            cabinet_model = Cabinet(door_params=np.array([width, height, 0.018, 0.4]), 
                                    axis_pos=axis_pos,
                                    T_A_S=T_A_S,
                                    save_path=config['cabinet_urdf_save_path'],
                                    has_handle=True)
            
            # Start Gazebo processes
            while True:
                kill_processes()
                gazebo_process = start_gazebo_launcher()
                print("Gazebo launcher started.")
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
            try:
                robot = UR5Commander()
            except RuntimeError as e:
                stop_gazebo_launcher(gazebo_process)
                continue

            rospy.sleep(1.)

            path_found = True
            success = robot.send_joint_values_to_robot(joint_values=q_traj[0], wait=True) # position to grasp
            if not success:
                stop_gazebo_launcher(gazebo_process)
                continue  

            # Spawning model in Gazebo
            cabinet_model.delete_model_gazebo()
            cabinet_model.spawn_model_gazebo()
            contact_sub = rospy.Subscriber('/contact', ContactsState, contact_callback)
            rospy.sleep(1.)

            while True: # loop if it does not manage to attach
                try:
                    # Attach handle to gripper
                    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
                    attach_srv.wait_for_service()
                    rospy.loginfo('Attaching handle to gripper')
                    req = AttachRequest()
                    req.model_name_1 = 'robot'
                    req.link_name_1 = 'wrist_3_link'
                    req.model_name_2 = 'my_cabinet'
                    req.link_name_2 = 'door_link'
                    attach_srv.call(req)
                    rospy.sleep(1.)
                    break
                except:
                    pass


            trajectory_successful = robot.send_multiple_joint_space_poses_to_robot2(q_traj[1:])

            cabinet_final_pose = get_link_pose('my_cabinet', 'door_link')
            T_A_S_final = pose_to_matrix(cabinet_final_pose)
            dist = np.linalg.norm(T_A_S_final[:3, 3] - T_A_S[:3, 3])

            # Check if the cabinet moved while executing trajectory
            trajectory_successful = True
            if dist > 0.02:
                trajectory_successful = False
                rospy.loginfo('The cabinet moved more than 2 cm.')

            # Check if door is open
            final_door_state = np.rad2deg(cabinet_model.get_door_state_gazebo()[1])
            print('Door angle: %f', final_door_state)
            door_opened = 85.0 <= abs(final_door_state) <= 95.0
            
            if trajectory_successful and contact_free and door_opened:
                print('Experiment finished successfully')
                final_success = True
            
            # Shutdown Gazebo simulation and kill all of its processes
            stop_gazebo_launcher(gazebo_process)

            if is_saving_results:
                with open(csv_path, 'a') as f:
                    writer = csv.writer(f, delimiter=',')
                    writer.writerow([path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])
            
            i += 1
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn("Time moved backwards. Skipping this loop iteration.")
