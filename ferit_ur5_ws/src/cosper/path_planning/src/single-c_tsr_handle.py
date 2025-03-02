#!/usr/bin/python

import rospy
import os
from core.util import read_config, read_csv_DataFrame
from core.ur5_commander import UR5Commander
from gazebo_push_open.cabinet_model import Cabinet
import numpy as np
from core.transforms import rot_z, pose_to_matrix
from gazebo_msgs.msg import ContactsState
import csv
from gazebo_msgs.msg import ContactsState
from rospkg import RosPack
from core.gazebo import get_link_pose
from tf2_ros import Buffer, TransformListener
from utils import *


if __name__ == '__main__':
    rospy.init_node('single-contact_tsr_handle_node')
    
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
    cfg_path = os.path.join(pkg_path, 'config/config_single-c_%s_handle_axis_left.yaml' % method_name)
    config = read_config(cfg_path)

    # Save/load path for results
    csv_path = config['results_path']

    # Gazebo simulation launch file (UR5 moveit config launch)
    gazebo_launch_file = config['gazebo_launch_path']

    # TSR trajectories path
    tsr_trajectories_path = config['tsr_trajectories_path']

    # Load door configurations
    door_configs_path = config['cabinet_configs_path']
    doors = np.load(door_configs_path)
    num_doors = doors.shape[0]
    
    # Screenshots path
    save_screenshot_path = config['screenshots_path']

    # If False, the data loads and the experiment starts where it stopped
    start_i = 0
    if START_FROM_BEGINNING:
        if IS_SAVING_RESULTS:
            with open(csv_path, 'w') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(['idx','path_found', 'traj_success', 'contact_free', 'door_opened', 'door_width', 'door_height', 'x', 'y', 'z', 'rot_z', 'state_angle', 'axis_pos'])
    else:
        data = read_csv_DataFrame(csv_path)
        start_i = data.shape[0]
        # doors = doors[rows:, :]

    # Transformation of robot base w.r.t world frame
    T_R_W = np.load(config['robot_world_pose'])

    i = start_i
    while i < num_doors:
        print('Cabinet %d' %i)
        try:
            path_found = False
            trajectory_successful = False
            door_opened = False
            contact_free = True
            final_success = False

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
                with open(csv_path, 'a') as f:
                    writer = csv.writer(f, delimiter=',')
                    writer.writerow([i, path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])
                    i+=1
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
            cabinet_model = Cabinet(door_params=np.array([width, height, 0.018, 0.3]), 
                                    axis_pos=axis_pos,
                                    T_A_S=T_A_S,
                                    save_path=config['cabinet_urdf_save_path'],
                                    has_handle=True)
            
            # Start Gazebo processes
            gazebo_process = start_gazebo_processes(gazebo_launch_file, tf_buffer, T_R_W)   
            
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
                if IS_SAVING_RESULTS:
                    with open(csv_path, 'a') as f:
                        writer = csv.writer(f, delimiter=',')
                        writer.writerow([i, path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])
                i += 1
                continue  

            # Spawning model in Gazebo
            cabinet_model.delete_model_gazebo()
            cabinet_model.spawn_model_gazebo()
            contact_state = {'contact_free': True}
            contact_sub = rospy.Subscriber('/contact', ContactsState, contact_callback, contact_state)
            rospy.sleep(2.)

            # Attach gripper to handle
            if not attach_two_models('robot', 'wrist_3_link', 'my_cabinet', 'door_link', 5.0):
                rospy.logerr("Attachment failed. Aborting...")
                rospy.signal_shutdown('No attaching.')
                continue

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

            if IS_SAVING_RESULTS:
                with open(csv_path, 'a') as f:
                    writer = csv.writer(f, delimiter=',')
                    writer.writerow([i, path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])
            
            i += 1
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(f"Time moved backwards. Exception: {e}")
            rospy.loginfo(f"Current ROS time: {rospy.Time.now()}")
            rospy.sleep(2)  # Let time stabilize