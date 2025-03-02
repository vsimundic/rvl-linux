#!/usr/bin/python

import rospy
import os
from core.util import read_config, read_csv_DataFrame
from core.ur5_commander import UR5Commander
from gazebo_push_open.cabinet_model import Cabinet
import numpy as np
from core.transforms import rot_z, pose_to_matrix
from gazebo_msgs.msg import ContactsState
from core.gazebo import get_link_pose
import csv
from gazebo_msgs.msg import ContactsState
from rospkg import RosPack
from tf2_ros import Buffer, TransformListener
from utils import *


if __name__ == '__main__':
    rospy.init_node('multi_contact_our_handleless_node')

    IS_SAVING_RESULTS = True
    IS_SAVING_IMAGES = False
    START_FROM_BEGINNING = False
    method_name = 'our'

    # TF buffer setup
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    rp = RosPack()
    pkg_path = rp.get_path('path_planning')
    
    # Config
    cfg_path = os.path.join(pkg_path, 'config/config_multi-c_%s_handleless_axis_left.yaml' % method_name)
    config = read_config(cfg_path)
    
    # Save/load path for results
    csv_path = config['results_path']

    # Gazebo simulation launch file (UR5 moveit config launch)
    gazebo_launch_file = config['gazebo_launch_path']

    # Load door configurations
    door_configs_path = config['cabinet_configs_path']
    doors = np.load(door_configs_path)
    num_doors = int(doors.shape[0])

    # Load screenshots folder
    save_screenshot_path = config['screenshots_path']

    # RVL config path
    rvl_cfg = config['rvl_config_path'] 

    # If False, the data loads and the experiment starts where it stopped
    start_i = 0
    if START_FROM_BEGINNING:
        if IS_SAVING_RESULTS:
            with open(csv_path, 'w') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(['idx','path_found', 'traj_success', 'contact_free', 'door_opened', 'door_width', 'door_height', 'x', 'y', 'z', 'rot_z', 'state_angle', 'axis_pos'])
    else:
        data = read_csv_DataFrame(csv_path)
        start_i = int(data.shape[0])
        # doors = doors[rows:, :]
    
    T_G_T = np.load(config['gripper_tool_pose'])
    
    T_R_W = np.load(config['robot_world_pose'])

    # Initial pose in joint space
    q_init = np.array([0., -np.pi*0.5, 0., -np.pi*0.5, 0., 0.])
    q_init_ros = q_init.copy()
    # Adjust joint values from ROS
    q_init[0] += np.pi
    q_init[5] += np.pi
    q_init[q_init>np.pi]-=(2.0*np.pi)
    q_init[q_init<-np.pi]+=(2.0*np.pi)

    # Static cabinet params
    door_thickness=config['cabinet_door_dims']['depth']
    static_depth=config['cabinet_door_dims']['static_depth']

    # Static mesh filename
    cabinet_static_mesh_filename = config['cabinet_static_mesh_save_path']
    cabinet_panel_mesh_filename = config['cabinet_panel_mesh_save_path']
    cabinet_full_mesh_filename = config['cabinet_full_mesh_save_path']

    i = start_i
    while i < num_doors:
        print('Cabinet %d' % i)
        try:
            path_found = False
            trajectory_successful = False
            door_opened = False
            contact_free = True
            final_success = False
            
            # Cabinet parameters
            door = doors[i, :]
            width = door[0]
            height = door[1]
            position = door[2:5]
            rot_z_deg = door[5]
            state_angle = door[6]
            axis_pos = door[7]

            T_A_S = np.eye(4)
            T_A_S[:3, 3] = np.array(position)
            # T_A_S[2, 3] += T_R_W[2, 3]
            Tz = np.eye(4)
            Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
            T_A_S = T_A_S @ Tz

            # Create a cabinet object
            cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                                    axis_pos=axis_pos,
                                    T_A_S=T_A_S,
                                    save_path=config['cabinet_urdf_save_path'],
                                    has_handle=False)
                    
            # Save cabinet mesh to a file
            cabinet_model.save_mesh_without_doors(cabinet_static_mesh_filename)
            cabinet_model.save_door_panel_mesh(cabinet_panel_mesh_filename)
            cabinet_model.change_door_angle(state_angle)
            cabinet_model.update_mesh()
            # Save full mesh
            cabinet_model.save_mesh(cabinet_full_mesh_filename)

            # RVL path planning
            T_G_0_array, q, all_feasible_paths, all_feasible_paths_q = rvl_path_planning(rvl_cfg, T_R_W, q_init, 37, False, cabinet_model, state_angle)
            
            if T_G_0_array.shape[0] == 1:
                print('Path not found')
                if IS_SAVING_RESULTS:
                    with open(csv_path, 'a') as f:
                        writer = csv.writer(f, delimiter=',')
                        writer.writerow([i, path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])

            if T_G_0_array.shape[0] > 1:

                path_found = True

                # Start Gazebo processes
                kill_ros_nodes()
                rospy.sleep(1.)
                kill_processes()
                reset_tf_buffer(tf_buffer)
                launch_process = subprocess.Popen("source /home/RVLuser/ferit_ur5_ws/devel/setup.bash && roslaunch ur5_robotiq_ft_3f_moveit_config demo_gazebo.launch", shell=True, executable="/bin/bash")
                rospy.sleep(4.)

                # Start the robot commander
                try:
                    robot = UR5Commander()
                except RuntimeError as e:
                    # stop_gazebo_launcher(gazebo_process)
                    kill_processes()
                    launch_process.terminate()
                    launch_process.wait()
                    continue

                # Sleep to ensure the robot is ready
                rospy.sleep(1.)

                # # Spawning model in Gazebo
                # cabinet_model.delete_model_gazebo()
                # cabinet_model.spawn_model_gazebo()

                # # Open doors in Gazebo
                # cabinet_model.set_door_state_gazebo(state_angle)
                # cabinet_model.change_door_angle(state_angle)

                contact_state = {'contact_free': True}
                contact_sub = rospy.Subscriber('/contact', ContactsState, contact_callback, contact_state)
                rospy.sleep(1.)
                
                # Points from the path planner
                # adjust joint values to ROS
                q[:, 0] -= np.pi
                q[:, 5] -= np.pi
                q[q>np.pi]-=(2.0*np.pi)     
                q[q<-np.pi]+=(2.0*np.pi)
                q = np.unwrap(q, axis=0)
                q = q.tolist()

                # Spawn the cabinet in Moveit
                # robot.add_mesh_to_scene(cabinet_full_mesh_filename, 'cabinet', cabinet_model.T_A_S)

                # Go to the first point
                robot.send_multiple_joint_space_poses_to_robot2(q[:2])
                # Spawning model in Gazebo
                cabinet_model.delete_model_gazebo()
                cabinet_model.spawn_model_gazebo()
                
                # Open doors in Gazebo
                cabinet_model.set_door_state_gazebo(state_angle)
                cabinet_model.change_door_angle(state_angle)
                rospy.sleep(1.)

                # # Remove from scene
                # robot.remove_from_scene('cabinet')

                # Execute the trajectory and wait until it finishes
                trajectory_successful = robot.send_multiple_joint_space_poses_to_robot2(q[1:])

                final_door_state = np.rad2deg(cabinet_model.get_door_state_gazebo()[1])
                print('Door angle: %f' % final_door_state)
                door_opened = 85.0 <= abs(final_door_state) <= 95.0

                # Check if there was contact
                contact_free = contact_state['contact_free']
                
                if trajectory_successful and contact_free and door_opened:
                    print('Experiment finished successfully')
                    final_success = True

                if IS_SAVING_RESULTS:
                    with open(csv_path, 'a') as f:
                        writer = csv.writer(f, delimiter=',')
                        writer.writerow([i, path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])

                # Shutdown Gazebo simulation and kill all of its processes
                # stop_gazebo_launcher(gazebo_process)
                # del gazebo_process
                launch_process.terminate()
                launch_process.wait()
                kill_processes()
                
            i += 1

        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(f"Time moved backwards. Exception: {e}")
            rospy.loginfo(f"Current ROS time: {rospy.Time.now()}")
            rospy.sleep(2)  # Let time stabilize


