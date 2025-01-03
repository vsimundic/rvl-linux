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

rospy.init_node('spawn_cabinet_node')

doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/door_configurations_axis_left.npy')

pkg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning'
doors_tsr_configs_path = os.path.join(pkg_path, 'tsr_data', 'grasp_open_configs')

moveit_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results_single_contact_handle_moveit.csv'
mesh_save_path = '/home/RVLuser/ferit_ur5_ws/test_cabinet.ply'
urdf_save_path = '/home/RVLuser/ferit_ur5_ws/test_cabinet.urdf'
moveit_df = read_csv_DataFrame(moveit_results_path)
successful_indices = moveit_df.index[((moveit_df['path_found'] == True) & 
                        (moveit_df['traj_success'] == True) & 
                        (moveit_df['contact_free'] == True) & 
                        (moveit_df['door_opened'] == True))].tolist()

# i_door = successful_indices[0]
i_door = 19

door = doors[i_door]

T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

door = doors[i_door, :]
width = door[0]
height = door[1]
state_angle = door[6]
axis_pos = door[7]
rot_z_deg = door[5]
position = door[2:5]

T_A_S = np.eye(4)
T_A_S[:3, 3] = np.array(position)
T_A_S[2, 3] += T_B_S[2, 3]
Tz = np.eye(4)
Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
T_A_S = T_A_S @ Tz

# Create a cabinet object
cabinet_model = Cabinet(door_params=np.array([width, height, 0.018, 0.4]), 
                        axis_pos=axis_pos,
                        T_A_S=T_A_S,
                        save_path=urdf_save_path,
                        has_handle=True)

cabinet_model.save_mesh(mesh_save_path)


robot = UR5Commander()
rospy.sleep(1.)
# Spawn model in PlanningSceneInterface
robot.remove_from_scene('cabinet')
T_O_B = np.linalg.inv(robot.T_B_S) @ cabinet_model.T_O_S
robot.add_mesh_to_scene(mesh_save_path, 'cabinet', T_O_B)

cabinet_model.delete_model_gazebo()
cabinet_model.spawn_model_gazebo()