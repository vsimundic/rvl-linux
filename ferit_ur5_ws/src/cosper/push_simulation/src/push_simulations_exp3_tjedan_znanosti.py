#!/usr/bin/python

import rospy
import os
import rospkg
from core.util import read_config, read_csv_DataFrame
from core.paths_packages import get_package_name_from_node, get_package_path_from_name
from core.ur5_commander import UR5Commander
from DDMan import push
from gazebo_push_open.cabinet_model import Cabinet
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
from core.transforms import rot_z, matrix_to_pose
import RVLPYDDManipulator as rvlpy_dd_man
from trac_ik_python.trac_ik import IK
import roslaunch
from gazebo_msgs.msg import ContactsState
from subprocess import check_output
import signal
import csv

contact_free = True

def contact_callback(msg: ContactsState):
    global contact_free

    if len(msg.states) > 0:
        contact_free = False

def get_pid(name: str):
    return list(map(int, check_output(['pidof', name]).split()))

def kill_gazebo_processes():
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
    rospy.init_node('node_simulations')
    
    cfg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/simulations_real_robot_config.yaml'
    config = read_config(cfg_path)
    csv_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_exp3.csv'
    doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/door_params_poses_exp3_axis_left.npy')
    start_from_beginning = False

    if start_from_beginning:
        # with open(csv_path, 'w') as f:
        #     writer = csv.writer(f, delimiter=',')
        #     writer.writerow(['idx','path_found', 'traj_success', 'contact_free', 'door_opened', 'door_width', 'door_height', 'x', 'y', 'z', 'rot_z', 'state_angle', 'axis_pos'])
        pass
    else:
        data = read_csv_DataFrame(csv_path)
        rows = data.shape[0]
        # doors = doors[rows:, :]
        doors = data

    
    T_B_S = np.eye(4)
    T_B_S[2, 3] = 0.005



    # Start Gazebo simulation
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"])
    launch.start()
    rospy.loginfo('Started Gazebo simulation')
    rospy.sleep(5)


    # for i in range(doors.shape[0]):
    for i in range(1000000):
        door = doors[7, :]
        print('\n')
        print(doors[7])
        print('\n')
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(door[2:5])
        T_A_S[2, 3] += T_B_S[2, 3]
        Tz_init = np.eye(4)
        Tz_init[:3, :3] = rot_z(np.radians(90.))
        # T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(door[ 5]))
        T_A_S = T_A_S @ Tz
        
        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array([door[0], door[1], 0.018, 0.4]), 
                                axis_pos=config['cabinet_pose']['axis_pos'],
                                T_A_S=T_A_S,
                                save_path=config['cabinet_urdf_save_path'])
                
        # Save cabinet mesh to a file
        cabinet_model.save_mesh_without_doors(config['cabinet_static_mesh_save_path'])

        # Path planning setup
        path_planner = rvlpy_dd_man.PYDDManipulator()
        path_planner.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec.cfg')
        path_planner.set_robot_pose(T_B_S)
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
        path_planner.set_environment_state(door[6])

        q_init = np.array([0., -1.5674883378518185, 0., -1.5676032728569234, 0., 0.])
        # adjust joint values from ROS
        q_init[0] += np.pi
        q_init[5] += np.pi
        q_init[q_init>np.pi]-=(2.0*np.pi)     
        q_init[q_init<-np.pi]+=(2.0*np.pi)

        # Plan 
        T_G_0_array, q = path_planner.path2(np.array(q_init), -90.0, 17, False)

        del path_planner

        path_found = False
        trajectory_successful = False
        door_opened = False
        contact_free = True
        final_success = False
        
        if T_G_0_array.shape[0] == 1:
            print('Path not found')
            # with open(csv_path, 'a') as f:
            #     writer = csv.writer(f, delimiter=',')
            #     # success, width, height, x, y, z, rot_z
            #     # writer.writerow([final_success, door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg])
                
            #     # path found, traj_success, contact detected, door opened, width, height, x, y, z, rot_z
            #     writer.writerow([path_found, trajectory_successful, contact_free, door_opened, door[ 0], door[ 1], door[ 2], door[ 3], door[ 4], door[ 5], door[ 6], door[ 7]])

        if T_G_0_array.shape[0] > 1:

            path_found = True


            # kill_gazebo_processes()

            # # Start Gazebo simulation
            # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            # # roslaunch.configure_logging(uuid)
            # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"])
            # launch.start()
            # rospy.loginfo('Started Gazebo simulation')
            # # rospy.sleep(5)
        

            # Robot handler
            robot = UR5Commander()

            # Spawning model in Gazebo
            cabinet_model.delete_model_gazebo()
            cabinet_model.spawn_model_gazebo()

            # Open doors in Gazebo
            # theta_deg = config['feasible_poses']['dd_state_deg']
            theta_deg = config['feasible_poses']['dd_state_deg']
            cabinet_model.set_door_state_gazebo(door[ 6])
            cabinet_model.change_door_angle(door[ 6])
            cabinet_model.update_mesh()

            contact_sub = rospy.Subscriber('/contact', ContactsState, contact_callback)
            
            # adjust joint values to ROS
            q[:, 0] += np.pi
            q[:, 5] += np.pi
            q[q>np.pi]-=(2.0*np.pi)     
            q[q<-np.pi]+=(2.0*np.pi)

            # Execute the trajectory and wait until it finishes
            # Time parametrization
            execute_time = 20
            # sum_ = 0
            # velocities = []
            # last_pt = T_G_0_array[0, :3, 3]
            # for i_ in range(T_G_0_array.shape[0]):
            #     dist = np.linalg.norm(T_G_0_array[i_, :3, 3] - last_pt)
            #     sum_ += dist
            #     last_pt = T_G_0_array[i_, :3, 3]
            #     velocities.append(sum_)
            
            # velocities /= sum_
            # velocities *= execute_time
            # trajectory_successful = robot.send_multiple_joint_space_poses_to_robot2(q, velocities, execute_time, wait=True)
            trajectory_successful = robot.send_multiple_joint_space_poses_to_robot(q, execute_time, wait=True)

            final_door_state = np.rad2deg(cabinet_model.get_door_state_gazebo()[1])
            print('Door angle: %f', final_door_state)
            door_opened = 85.0 <= abs(final_door_state) <= 95.0
            if trajectory_successful and contact_free and door_opened:
                print('Experiment finished successfully')
                final_success = True
            

            robot.send_named_pose('up')
        
            # Shutdown Gazebo simulation and kill all its processes
            # launch.shutdown()
            # rospy.sleep(5)

            # with open(csv_path, 'a') as f:
            #     writer = csv.writer(f, delimiter=',')
            #     # success, width, height, x, y, z, rot_z
            #     # writer.writerow([final_success, door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg])
                
            #     # path found, traj_success, contact detected, door opened, width, height, x, y, z, rot_z
            #     print(door[ 0], door[ 1], door[ 2], door[ 3], door[ 4], door[ 5], door[ 6], door[ 7])
            #     writer.writerow([path_found, trajectory_successful, contact_free, door_opened, door[ 0], door[ 1], door[ 2], door[ 3], door[ 4], door[ 5], door[ 6], door[ 7]])



