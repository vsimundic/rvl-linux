#!/usr/bin/python

import rospy
import os
from core.util import read_config, read_csv_DataFrame
from core.ur5_commander import UR5Commander
from cabinet_model import Cabinet
import numpy as np
from core.transforms import rot_z
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
np.random.seed(69)

contact_free = True

def contact_callback(msg: ContactsState):
    global contact_free

    if len(msg.states) > 0:
        contact_free = False

def get_pid(name: str):
    return list(map(int, check_output(['pidof', name]).split()))

def kill_processes():
    try:
        gzserver_pids = get_pid('gzserver')
        if len(gzserver_pids) > 0:
            for pid in gzserver_pids:
                os.kill(pid, signal.SIGKILL)
    except Exception as e:
        pass
    try:
        gzclient_pids = get_pid('gzclient')
        if len(gzclient_pids) > 0:
            for pid in gzclient_pids:
                os.kill(pid, signal.SIGKILL)
    except Exception as e:
        pass
    try:
        rviz_pids = get_pid('rviz')
        if len(rviz_pids) > 0:
            for pid in rviz_pids:
                os.kill(pid, signal.SIGKILL)
    except Exception as e:
        pass
    try:
        rviz_pids = get_pid('move_group')
        if len(rviz_pids) > 0:
            for pid in rviz_pids:
                os.kill(pid, signal.SIGKILL)
    except Exception as e:
        pass
    try:
        rviz_pids = get_pid('robot_state_pub')
        if len(rviz_pids) > 0:
            for pid in rviz_pids:
                os.kill(pid, signal.SIGKILL)
    except Exception as e:
        pass

if __name__ == '__main__':
    rospy.init_node('node_simulation_single_contact_experiment')
    
    rp = RosPack()
    pkg_path = rp.get_path('path_planning')

    is_saving_results = True

    # Config
    cfg_path = os.path.join(pkg_path, 'config/config_simulations_axis_left.yaml')
    config = read_config(cfg_path)

    # Save/load path for results
    csv_path = os.path.join(pkg_path,'results_simulation_single_contact.csv')

    # Load door configurations
    door_configs_path = os.path.join(pkg_path, 'door_configurations_axis_left.npy')
    doors = np.load(door_configs_path)
    
    is_saving_images = False
    save_screenshot_path = os.path.join(pkg_path, 'single_contact_screenshots')

    # If False, the data loads and the experiment starts where it stopped
    start_from_beginning = True
    if start_from_beginning:
        if is_saving_results:
            with open(csv_path, 'w') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(['path_found', 'traj_success', 'contact_free', 'door_opened', 'door_width', 'door_height', 'x', 'y', 'z', 'rot_z', 'state_angle', 'axis_pos'])
    else:
        data = read_csv_DataFrame(csv_path)
        rows = data.shape[0]
        doors = doors[rows:, :]

    T_G_T = np.load(os.path.join(pkg_path, 'config/T_G_T.npy'))
    
    T_R_W = np.eye(4)
    T_R_W[2, 3] = 0.005

    # Initial pose in joint space
    q_init = np.array([0., -1.5674883378518185, 0., -1.5676032728569234, 0., 0.])
    q_init_ros = q_init.copy()
    # adjust joint values from ROS
    q_init[0] += np.pi
    q_init[5] += np.pi
    q_init[q_init>np.pi]-=(2.0*np.pi)
    q_init[q_init<-np.pi]+=(2.0*np.pi)

    for i in range(doors.shape[0]):
        door = doors[i, :]
        width = door[0]
        height = door[1]
        state_angle = door[6]
        axis_pos = door[7]
        rot_z_deg = door[5]
        position = door[2:5]


        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(position)
        T_A_S[2, 3] += T_R_W[2, 3]
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
        T_A_S = T_A_S @ Tz
        
        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array([width, height, 0.018, 0.4]), 
                                axis_pos=config['cabinet_pose']['axis_pos'],
                                T_A_S=T_A_S,
                                save_path=config['cabinet_urdf_save_path'])
                
        # Save cabinet mesh to a file
        cabinet_model.save_mesh_without_doors(config['cabinet_mesh_save_path'])
        cabinet_model.change_door_angle(state_angle)
        cabinet_model.update_mesh()

        # Path planning setup
        path_planner = rvlpy_dd_man.PYDDManipulator()
        path_planner.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_exp1.cfg')
        path_planner.set_robot_pose(T_R_W)
        path_planner.set_door_model_params(
                                        cabinet_model.d_door,
                                        cabinet_model.w_door,
                                        cabinet_model.h_door,
                                        0.0, # rx
                                        -(cabinet_model.w_door/2. - cabinet_model.axis_distance), # ry
                                        cabinet_model.axis_pos, # opening direction
                                        cabinet_model.static_side_width,
                                        cabinet_model.moving_to_static_part_distance)
        path_planner.set_door_pose(T_A_S)
        path_planner.set_environment_state(state_angle)

        # Plan
        try:
            T_G_0_array, q, all_feasible_paths, all_feasible_paths_q = path_planner.path2(np.array(q_init), -90.0, 1, True)
        except (_ArrayMemoryError, ValueError) as e:
            T_G_0_array = np.zeros((1,4,4))

        del path_planner
        
        path_found = False
        trajectory_successful = False
        door_opened = False
        contact_free = True
        final_success = False
        
        if T_G_0_array.shape[0] == 1:
            print('Path not found')
            with open(csv_path, 'a') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow([path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])

        if T_G_0_array.shape[0] > 1:

            path_found = True
            mask = all_feasible_paths[:, 3, 3, 3] > 0.5
            all_feasible_paths = all_feasible_paths[mask]
            # Take a random path
            num_paths = all_feasible_paths.shape[0]
            random_path_num = np.random.randint(0, num_paths)
            feasible_path = all_feasible_paths[random_path_num, :, :, :]
            
            # Generate poses from the first state (opened by a state_angle value)
            T_G_S_unlatched = T_R_W @ feasible_path[3]
            T_A_S_unlatched = cabinet_model.T_O_S @ cabinet_model.T_A_O
            T_G_A = np.linalg.inv(T_A_S_unlatched)  @ T_G_S_unlatched

            n_path_points = 4+16
            n_poses = 17        
            opening_angles = np.linspace(state_angle, axis_pos*90., num=n_poses)
            
            T_G_S_path = np.zeros((n_poses, 4, 4))
            for i_pose, angle_ in enumerate(opening_angles):
                T_A_O = cabinet_model.change_door_angle(angle_)
                T_A_S_ = cabinet_model.T_O_S @ T_A_O
                T_G_S_path[i_pose, :, :] = T_A_S_ @ T_G_A
            
            # Reset the angle
            cabinet_model.change_door_angle(state_angle)
            
            T_G_0_path = np.zeros((n_path_points, 4, 4))
            T_G_0_path[0:4, :, :] = feasible_path
            T_G_0_path[-n_poses:, :, :] = np.linalg.inv(T_R_W)[np.newaxis, :, :] @ T_G_S_path
            T_T_0_path = T_G_0_path @ np.linalg.inv(T_G_T)[np.newaxis, :, :]

            kill_processes()

            # Start Gazebo simulation
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"])
            launch.start()
            rospy.loginfo('Started Gazebo simulation')
            rospy.sleep(5.)
            # Robot handler
            robot = UR5Commander()


            # Calculate joint values for all points of the path
            q_path = []
            q_temp = q_init_ros.tolist()
            for i_q in range(n_path_points):
                q_ = robot.get_inverse_kin(q_temp, T_T_0_path[i_q, :, :])

                if q_ is None:
                    # path_found = False
                    # break
                    continue
                
                q_path.append(q_)
                q_temp = q_

            if not path_found:
                if is_saving_results:
                    with open(csv_path, 'a') as f:
                        writer = csv.writer(f, delimiter=',')
                        writer.writerow([path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])
                launch.shutdown()
                continue

            # Spawning model in Gazebo
            cabinet_model.delete_model_gazebo()
            cabinet_model.spawn_model_gazebo()

            # Open doors in Gazebo
            cabinet_model.set_door_state_gazebo(state_angle)
            cabinet_model.change_door_angle(state_angle)
            cabinet_model.update_mesh()

            contact_sub = rospy.Subscriber('/contact', ContactsState, contact_callback)
            
            # Execute the trajectory and wait until it finishes
            # Time parametrization
            execute_time = 20

            collisions = np.zeros((len(q_path), 3))
            for i_q in range(len(q_path)):
                robot.send_joint_values_to_robot(joint_values=q_path[i_q], wait=True)
                contact_msg = rospy.wait_for_message('/contact_door', ContactsState, rospy.Duration(1))
                
                if len(contact_msg.states) > 0:
                    position = contact_msg.states[0].contact_positions[0]
                    t_P_S = np.array([position.x, position.y, position.z])
                    current_angle = cabinet_model.get_door_state_gazebo()[1]
                    T_A_O = cabinet_model.change_door_angle(np.rad2deg(current_angle))
                    T_A_S_rot = cabinet_model.T_O_S @ T_A_O
                    T_S_D = np.linalg.inv(cabinet_model.T_D_A) @ np.linalg.inv(T_A_S_rot)
                    t_P_D = np.dot(T_S_D[:3, :3], t_P_S) + T_S_D[:3, 3]
                    collisions[i_q, :] = t_P_D
                else:
                    collisions[i_q, :] = np.array([0.,0.,0.])
                rospy.sleep(0.5)

                if is_saving_images:
                    screenshot_saver = ImageGrab.grab()
                    try:
                        screenshot_saver.save(os.path.join(save_screenshot_path, '%d.png' % i_q))
                    except Exception as e:
                        print('Cannot save the image.')
                    screenshot_saver.close()
           
            print(collisions)

            final_door_state = np.rad2deg(cabinet_model.get_door_state_gazebo()[1])
            print('Door angle: %f', final_door_state)
            door_opened = 85.0 <= abs(final_door_state) <= 95.0
            if trajectory_successful and contact_free and door_opened:
                print('Experiment finished successfully')
                final_success = True
            
            # Shutdown Gazebo simulation and kill all its processes
            launch.shutdown()
            # rospy.sleep(5)


            if is_saving_results:
                with open(csv_path, 'a') as f:
                    writer = csv.writer(f, delimiter=',')
                    writer.writerow([path_found, trajectory_successful, contact_free, door_opened, width, height, position[0], position[1], position[2], rot_z_deg, state_angle, axis_pos])



