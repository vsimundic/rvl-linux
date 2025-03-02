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
from tf2_ros import Buffer, TransformListener
from rospy.exceptions import ROSException
from rosgraph_msgs.msg import Clock
# from path_planning.utilities import read_joint_values_from_csv, contact_callback, get_pid, kill_processes, wait_for_clock, reset_tf_buffer
from utils import read_joint_values_from_csv, contact_callback, start_gazebo_processes, stop_gazebo_launcher, attach_two_models

if __name__ == '__main__':
    rospy.init_node('single_contact_exp_tsr')
    
    IS_SAVING_RESULTS = True
    IS_SAVING_IMAGES = False
    START_FROM_BEGINNING = True
    method_name = 'tsr'

    # TF buffer setup
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    rp = RosPack()
    pkg_path = rp.get_path('path_planning')

    # Config
    cfg_path = os.path.join(pkg_path, 'config/config_{method_name}_simulations_axis_left.yaml')
    config = read_config(cfg_path)
    # Save/load path for results
    csv_path = os.path.join(pkg_path,'results_simulation_{method_name}_single_contact.csv')

    # Gazebo simulation launch file (UR5 moveit config launch)
    gazebo_launch_file = config['gazebo_launch_path']

    # TSR trajectories path
    tsr_trajectories_path = config['tsr_trajectories_path']

    # Load door configurations
    door_configs_path = os.path.join(pkg_path, 'door_configurations_axis_left.npy')
    doors = np.load(door_configs_path)
    num_doors = doors.shape[0]
    
    # Screenshots path
    save_screenshot_path = os.path.join(pkg_path, 'single_contact_screenshots_{method_name}')

    # # If False, the data loads and the experiment starts where it stopped
    # start_i = 0
    # if START_FROM_BEGINNING:
    #     if IS_SAVING_RESULTS:
    #         with open(csv_path, 'w') as f:
    #             writer = csv.writer(f, delimiter=',')
    #             writer.writerow(['idx','path_found', 'traj_success', 'contact_free', 'door_opened', 'door_width', 'door_height', 'x', 'y', 'z', 'rot_z', 'state_angle', 'axis_pos'])
    # else:
    #     data = read_csv_DataFrame(csv_path)
    #     start_i = data.shape[0]
    #     # doors = doors[rows:, :]


    data = read_csv_DataFrame(csv_path)

    data = data.loc[((data['path_found'] == True) & 
                            (data['traj_success'] == False) & 
                            (data['contact_free'] == True) & 
                            (data['door_opened'] == True))] 


    T_R_W = np.eye(4)
    T_R_W[2, 3] = 0.005

    for i, row in data.iterrows():
    # while i < num_doors:
        print('Cabinet %d' %i)
        try:
            door = doors[i, :]
            width = door[0]
            height = door[1]
            position = door[2:5]
            rot_z_deg = door[5]
            state_angle = door[6]
            axis_pos = door[7]

            traj_filename = os.path.join(tsr_trajectories_path, 'traj_%d.csv' %i)
            q_traj = read_csv_DataFrame(traj_filename)
            if q_traj is None or q_traj.empty:
                # with open(csv_path, 'a') as f:
                #     writer = csv.writer(f, delimiter=',')
                #     writer.writerow([i, path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])
                continue

            # convert to list of lists
            q_traj = q_traj.to_numpy().tolist()

            T_A_S = np.eye(4)
            T_A_S[:3, 3] = np.array(position)
            # T_A_S[2, 3] += T_R_W[2, 3]
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
            gazebo_process = start_gazebo_processes(gazebo_launch_file, tf_buffer)   
            
            # Start the robot commander
            try:
                robot = UR5Commander()
            except RuntimeError as e:
                stop_gazebo_launcher(gazebo_process)
                continue

            # Sleep to ensure the robot is ready
            rospy.sleep(2.)

            path_found = True
            success = robot.send_joint_values_to_robot(joint_values=q_traj[0], wait=True) # position to grasp
            if not success:
                stop_gazebo_launcher(gazebo_process)
                i += 1
                continue  

            # Spawning model in Gazebo
            cabinet_model.delete_model_gazebo()
            cabinet_model.spawn_model_gazebo()
            contact_state = {'contact_free': True}
            contact_sub = rospy.Subscriber('/contact', ContactsState, contact_callback, contact_state)
            rospy.sleep(2.)

            if not attach_two_models('robot', 'wrist_3_link', 'my_cabinet', 'door_link', 5.0):
                rospy.logerr("Attachment failed. Aborting...")
                rospy.signal_shutdown('No attaching.')


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

            # Check if there was contact
            contact_free = contact_state['contact_free']

            if trajectory_successful and contact_free and door_opened:
                print('Experiment finished successfully')
                final_success = True
            
            # Shutdown Gazebo simulation and kill all of its processes
            stop_gazebo_launcher(gazebo_process)

        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(f"Time moved backwards. Exception: {e}")
            rospy.loginfo(f"Current ROS time: {rospy.Time.now()}")
            rospy.sleep(2)  # Let time stabilize