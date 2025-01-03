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
import yaml 
from tqdm import tqdm
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
    rospy.init_node('single_contact_exp_handle_moveit')
    
    rp = RosPack()
    pkg_path = rp.get_path('path_planning')

    is_saving_results = True

    # Config
    cfg_path = os.path.join(pkg_path, 'config/config_simulations_axis_left.yaml')
    config = read_config(cfg_path)

    # Load door configurations
    door_configs_path = os.path.join(pkg_path, 'door_configurations_axis_left.npy')
    doors = np.load(door_configs_path)
    
    is_saving_images = False
    save_screenshot_path = os.path.join(pkg_path, 'single_contact_screenshots')

    tsr_configs_path = os.path.join(pkg_path, 'tsr_data', 'cabinet_configs')
    moveit_traj_save_path = os.path.join(pkg_path, 'moveit_traj_data')

    # Start Gazebo sim
    kill_processes()
    kill_processes()
    kill_processes()
    rospy.sleep(1.)
    # Start Gazebo simulation
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"])
    launch.start()
    rospy.loginfo('Started Gazebo simulation')
    rospy.sleep(5.)
    # Robot handler
    robot = UR5Commander()
    rospy.sleep(2.)

    # If False, the data loads and the experiment starts where it stopped
    start_i=0

    T_R_W = np.eye(4)
    T_R_W[2, 3] = 0.005

    q_init = [0.0, float(np.deg2rad(-90.)), 0.0, float(np.deg2rad(-90.)), 0.0, 0.0]

    str_w = ['j0','j1','j2','j3','j4','j5']
    init_config = ""

    for i in tqdm(range(start_i, doors.shape[0])):

        door = doors[i, :]
        width = door[0]
        height = door[1]
        state_angle = door[6]
        axis_pos = door[7]
        rot_z_deg = door[5]
        position = door[2:5]


        f_traj = open(os.path.join(moveit_traj_save_path, 'traj_%d.csv' %i), 'r')
        head = f_traj.readline()
        

        tsr_config_filename = os.path.join(tsr_configs_path, 'cabinet_%d.yaml'%i)
        with open(tsr_config_filename, 'r') as f:
            data = yaml.safe_load(f)
            init_config = data['initial_config']
            dist = np.sum(abs(np.array(q_init) - np.array(init_config)))
            if dist < 0.1:
                print('Path not found')
                f_traj.close()
                continue


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
        # Save cabinet mesh to a file
        cabinet_model.update_mesh()
        
        Tw_e = cabinet_model.T_H_A.copy()
        Tw_e[:3, :3] = np.array([[0, 0, 1],
                                [-1, 0, 0],
                                [0, -1, 0]])
        Tw_e[0, 3] -= 0.28
        Tz = np.eye(4)
        Tz[:3,:3] = rot_z(np.radians(45.)) # rotate gripper 45 deg to align fingers with handle
        Tw_e = Tw_e @ Tz
        T_T_A = Tw_e

        if True:

            n_path_points = 20
            opening_angles = np.linspace(0, axis_pos*90., num=n_path_points)
            
            Tz_ = np.eye(4)
            q_traj = [q_init, init_config]

            # Calculate joint values for all points of the path
            for i_pose, angle_ in enumerate(opening_angles[1:]):
                Tz_[:3,:3] = rot_z(np.deg2rad(angle_))
                T_A_S_ = T_A_S @ Tz_
                T_T_B_ = np.linalg.inv(robot.T_B_S) @ T_A_S_ @ T_T_A
                q_goal, _ = robot.get_inverse_kin_builtin(q_traj[-1], T_T_B_)

                if q_goal is not None:
                    q_traj.append(q_goal)
            
            # traj_writer.writerows(q_traj)
            f_traj.close()

