#!/usr/bin/env python

import yaml
import numpy as np
from core.util import read_csv_DataFrame
import os
import rospy
from gazebo_push_open.cabinet_model import Cabinet
from core.transforms import rot_z, rot_y,  matrix_to_pose
from core.ur5_commander import UR5Commander
from tqdm import tqdm

np.random.seed(69)

doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/door_configurations_axis_left.npy')

pkg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning'
doors_tsr_configs_path = os.path.join(pkg_path, 'tsr_data', 'full_open_configs')
urdf_path = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.urdf'

if not os.path.exists(doors_tsr_configs_path):
    os.makedirs(doors_tsr_configs_path)

T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

for i_door in tqdm(range(doors.shape[0])):
    
    door = doors[i_door]

    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array(door[2:5])
    T_A_S[2, 3] += T_B_S[2, 3]

    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.radians(door[ 5]))
    T_A_S = T_A_S @ Tz
    axis_pos = door[-1]
    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array([door[0], door[1], 0.018, 0.4]), 
                            axis_pos=axis_pos,
                            T_A_S=T_A_S,
                            save_path=os.path.join(doors_tsr_configs_path, 'cabinet_%d.urdf'%i_door),
                            has_handle=True)
    cabinet_mesh_filename = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.ply'

    cabinet_model.save_mesh(os.path.join(doors_tsr_configs_path, 'cabinet_%d.stl'%i_door))

    config_filename = 'cabinet_%d.yaml' % i_door


    ### TSR matrices definition ###
    T0_w = T_A_S
    T0_w_pose =  matrix_to_pose(T0_w)
    t0_w = T0_w_pose.position
    q0_w = T0_w_pose.orientation


    # T_6_H = np.eye(4)
    # T_6_H[:3, :3] = np.array([[0, 0, -axis_pos],
    #                             [axis_pos, 0, 0],
    #                             [0, -1, 0]])

    # For grasp, this is handle to hinge transform
    Tw_e = cabinet_model.T_H_A.copy()
    Tw_e[:3, :3] = np.array([[0, 0, -axis_pos],
                                [axis_pos, 0, 0],
                                [0, -1, 0]])
    Tw_e[0, 3] -= 0.02
    
    # Tw_e[0, 3] -= 0.28
    Tz = np.eye(4)
    Tz[:3,:3] = rot_z(np.radians(45.))
    Tw_e = Tw_e @ Tz
    Tw_e_pose =  matrix_to_pose(Tw_e)
    tw_e = Tw_e_pose.position
    qw_e = Tw_e_pose.orientation


    T_O_S = cabinet_model.T_O_S
    T_O_S_pose =  matrix_to_pose(T_O_S)
    t_o_s = T_O_S_pose.position
    q_o_s = T_O_S_pose.orientation


    T_T_A = Tw_e
    T_T_B_goal = np.linalg.inv(T_B_S) @ cabinet_model.T_A_S @ T_T_A

    initial_config = [0.0, float(np.deg2rad(-90.)), 0.0, float(np.deg2rad(-90.)), 0.0, 0.0]
    goal_config = [0.,0.,0.,0.,0.,0.]
    
    t_o_s_dict = dict()
    t_o_s_dict.update({'translation': [float(el) for el in [t_o_s.x, t_o_s.y, t_o_s.z]]})
    t_o_s_dict.update({'rotation': [float(el) for el in [q_o_s.w, q_o_s.x, q_o_s.y, q_o_s.z]]})

    config_data = dict()
    config_data.update({'scene_file': '/home/user/tsr_ws/ur5.dae'})
    config_data.update({'ground_file': '/home/user/tsr_ws/ground.xml'})
    config_data.update({'robot_name': 'ur5_robotiq_ft_3f'})
    config_data.update({'dof': 6})
    config_data.update({'initial_config': initial_config})
    config_data.update({'goal_config': goal_config})
    config_data.update({'psample': 0.5})
    config_data.update({'T_O_S': t_o_s_dict})
    
    t0_w_dict = dict()
    t0_w_dict.update({'translation': [float(el) for el in [t0_w.x, t0_w.y, t0_w.z]]})
    t0_w_dict.update({'rotation': [float(el) for el in [q0_w.w, q0_w.x, q0_w.y, q0_w.z]]})
    tw_e_dict = dict()
    tw_e_dict.update({'translation': [float(el) for el in [tw_e.x, tw_e.y, tw_e.z]]})
    tw_e_dict.update({'rotation': [float(el) for el in [qw_e.w, qw_e.x, qw_e.y, qw_e.z]]})
    
    
    
    tsr_dict = {'manipind': 0,
                'relativebodyname': 'NULL',
                'relativelinkname': 'NULL',
                'T0_w': t0_w_dict, 
                'Tw_e': tw_e_dict,
                'Bw': [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]],
                'sample_goal_from_chain': True,
                'sample_start_from_chain': False,
                'constrain_to_chain': True,
                'mimicbodyname': 'NULL'}
    
    config_data.update({'tsr': tsr_dict})

    with open(os.path.join(doors_tsr_configs_path, config_filename), 'w') as f:
        yaml.dump(config_data, f,  default_flow_style=None)

