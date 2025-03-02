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
    rospy.init_node('node_simulations')
    
    cfg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/simulations_exp3_axis_left.yaml'
    config = read_config(cfg_path)



    read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_exp3.csv'
    data = read_csv_DataFrame(read_results_path)

    data = data.loc[data['path_found'] == False]
    T_B_S = np.eye(4)
    T_B_S[2, 3] = 0.005

    print(data)

    kill_gazebo_processes()

    # Start Gazebo simulation
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"])
    launch.start()
    rospy.loginfo('Started Gazebo simulation')
    rospy.sleep(5)
    

    for i, row in data.iterrows():

        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array([row['x'], row['y'], row['z']])
        T_A_S[2, 3] += T_B_S[2, 3]
        Tz_init = np.eye(4)
        # Tz_init[:3, :3] = rot_z(np.radians(90.))
        # T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(row['rot_z']))
        T_A_S = T_A_S @ Tz

        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array([row['door_width'], row['door_height'], 0.018, 0.4]), 
                                axis_pos=row['axis_pos'],
                                T_A_S=T_A_S,
                                save_path=config['cabinet_urdf_save_path'])
                
        # Save cabinet mesh to a file
        cabinet_model.save_mesh_without_doors(config['cabinet_static_mesh_save_path'])


        # Spawning model in Gazebo
        cabinet_model.delete_model_gazebo()
        cabinet_model.spawn_model_gazebo()

        # Open doors in Gazebo
        theta_deg = row['state_angle']
        cabinet_model.set_door_state_gazebo(theta_deg)
        cabinet_model.change_door_angle(theta_deg)
        cabinet_model.update_mesh()

        key = input('press p: ')
           
    # Shutdown Gazebo simulation and kill all its processes
    launch.shutdown()
    rospy.sleep(5)



