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

if __name__ == '__main__':
    rospy.init_node('node_load_simulation_experiment_from_csv')

    rp = RosPack()
    pkg_path = rp.get_path('path_planning')

    # Config
    cfg_path = os.path.join(pkg_path, 'config/config_simulations_axis_left.yaml')
    config = read_config(cfg_path)

    is_saving_images = False
    save_screenshot_path = os.path.join(pkg_path, 'single_contact_screenshots')

    lock_T_G_DD = True

    # read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_exp1_final.csv'
    read_results_path = os.path.join(pkg_path,'results_simulation_single_contact.csv')
    data = read_csv_DataFrame(read_results_path)

    data = data.loc[((data['path_found'] == True) & 
                            (data['traj_success'] == True) & 
                            (data['contact_free'] == True) & 
                            (data['door_opened'] == True))] 
    
    T_R_W = np.eye(4)
    T_R_W[2, 3] = 0.005

    for i, row in data.iterrows():
        # row = data.iloc[16]
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array([row['x'], row['y'], row['z']])
        T_A_S[2, 3] += T_R_W[2, 3]
        Tz_init = np.eye(4)
        Tz_init[:3, :3] = rot_z(np.radians(90.))
        # T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(row['rot_z']))
        T_A_S = T_A_S @ Tz

        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array([row['door_width'], row['door_height'], 0.018, 0.4]), 
                                axis_pos=-1,
                                T_A_S=T_A_S,
                                save_path=config['cabinet_urdf_save_path'])
                
        # Save cabinet mesh to a file
        cabinet_model.save_mesh_without_doors(config['cabinet_static_mesh_save_path'])

        # Path planning setup
        path_planner = rvlpy_dd_man.PYDDManipulator()
        if lock_T_G_DD:
            path_planner.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_exp1.cfg')
        else:
            path_planner.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec.cfg')
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
        path_planner.set_environment_state(row['state_angle'])

        q_init = np.array([0., -1.5674883378518185, 0., -1.5676032728569234, 0., 0.])
        # adjust joint values from ROS
        q_init[0] += np.pi
        q_init[5] += np.pi
        q_init[q_init>np.pi]-=(2.0*np.pi)     
        q_init[q_init<-np.pi]+=(2.0*np.pi)

        # Plan 
        # T_G_0_array, q = path_planner.path2(np.array(q_init))
        # T_G_0_array, q, all_feasible_paths, all_feasible_paths_q = path_planner.path2(q_init, -90.0, 17, True)
        T_G_0_array, q = path_planner.path2(q_init, -90.0, 17, False)

        del path_planner

        path_found = False
        trajectory_successful = False
        door_opened = False
        contact_free = True
        final_success = False
        
        if T_G_0_array.shape[0] == 1:
            print('Path not found')
        if T_G_0_array.shape[0] > 1:

            path_found = True

            kill_gazebo_processes()

            # Start Gazebo simulation
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"])
            launch.start()
            rospy.loginfo('Started Gazebo simulation')
            rospy.sleep(5)
        

            # Robot handler
            robot = UR5Commander()

            # Spawning model in Gazebo
            cabinet_model.delete_model_gazebo()
            cabinet_model.spawn_model_gazebo()

            # Open doors in Gazebo
            theta_deg = config['feasible_poses']['dd_state_deg']
            cabinet_model.set_door_state_gazebo(row['state_angle'])
            cabinet_model.change_door_angle(row['state_angle'])
            cabinet_model.update_mesh()

            contact_sub = rospy.Subscriber('/contact', ContactsState, contact_callback)
            
            # adjust joint values to ROS
            q[:, 0] += np.pi
            q[:, 5] += np.pi
            q[q>np.pi]-=(2.0*np.pi)     
            q[q<-np.pi]+=(2.0*np.pi)

            # # Execute the trajectory and wait until it finishes
            # trajectory_successful = robot.send_multiple_joint_space_poses_to_robot(q, execute_time=20, wait=True)
            collisions = np.zeros((q.shape[0], 3))
            for i_q in range(q.shape[0]):
                robot.send_joint_values_to_robot(joint_values=q[i_q, :].tolist(), wait=True)
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
                    pass
                else:
                    collisions[i_q, :] = np.array([0.,0.,0.])
                rospy.sleep(0.5)

                if is_saving_images:
                    screenshot_saver = ImageGrab.grab()
                    screenshot_saver.save(os.path.join(save_screenshot_path, '%d.png' % i_q))
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
            rospy.sleep(5)



