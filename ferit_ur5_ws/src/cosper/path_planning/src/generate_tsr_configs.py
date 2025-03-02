#!/usr/bin/env python

import yaml
import numpy as np
import os
from gazebo_push_open.cabinet_model import Cabinet
from core.transforms import rot_z,  matrix_to_pose
from tqdm import tqdm

# np.random.seed(69)

doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/cabinet_configurations_axis_left.npy')

pkg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning'
doors_tsr_configs_path = os.path.join(pkg_path, 'tsr_data', 'cabinet_configs')
urdf_path = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.urdf'

if not os.path.exists(doors_tsr_configs_path):
    os.makedirs(doors_tsr_configs_path)


T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

door_thickness=0.018
static_depth=0.3


for i_door in tqdm(range(doors.shape[0])):
# for i_door in range(2):

    # Cabinet parameters
    door = doors[i_door, :]
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
                            save_path=urdf_path,
                            has_handle=True)

    config_filename = 'cabinet_%d.yaml' % i_door


    ### TSR matrices definition ###
    T0_w = T_A_S
    T0_w_pose = matrix_to_pose(T0_w)
    t0_w = T0_w_pose.position
    q0_w = T0_w_pose.orientation

    # print("  T0_w:")
    # print('    translation: [%f, %f, %f]' % (t0_w.x, t0_w.y, t0_w.z))
    # print('    rotation: [%f, %f, %f, %f] #wxyz' % (q0_w.w, q0_w.x, q0_w.y, q0_w.z))
    
    Tz45 = np.eye(4)
    Tz45[:3,:3] = rot_z(np.radians(45.))

    T_6_H = np.eye(4)
    T_6_H[:3, :3] = np.array([[0, 0, -axis_pos],
                                [axis_pos, 0, 0],
                                [0, -1, 0]])
    T_6_H = T_6_H @ Tz45
    T_6_H[0, 3] -= 0.28

    Tw_e = cabinet_model.T_H_A @ T_6_H
    Tw_e_pose =  matrix_to_pose(Tw_e)
    tw_e = Tw_e_pose.position
    qw_e = Tw_e_pose.orientation
    
    Tz90 = np.eye(4)
    Tz90[:3,:3] = rot_z(-np.pi/2.)

    T_H_S = T_A_S @ cabinet_model.T_H_A
    T_HO_S = T_A_S @ Tz90 @ cabinet_model.T_H_A

    # tool in world when open
    T_6O_S = T_HO_S @ T_6_H 

    # tool in open state in closed handle frame
    T_6O_H = np.linalg.inv(T_H_S) @ T_6O_S
    # Tw_e2 = T_6O_H
    T0_w2 = T_HO_S
    Tw_e2 = T_6_H

    Tw_e2_pose = matrix_to_pose(Tw_e2)
    t_w_e2 = Tw_e2_pose.position
    q_w_e2 = Tw_e2_pose.orientation
    
    T0_w2_pose = matrix_to_pose(T0_w2)
    t_0_w2 = T0_w2_pose.position
    q_0_w2 = T0_w2_pose.orientation


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
    config_data.update({'psample': 0.0})
    config_data.update({'T_O_S': t_o_s_dict})
    
    t0_w_dict = dict()
    t0_w_dict.update({'translation': [float(el) for el in [t0_w.x, t0_w.y, t0_w.z]]})
    t0_w_dict.update({'rotation': [float(el) for el in [q0_w.w, q0_w.x, q0_w.y, q0_w.z]]})
    tw_e_dict = dict()
    tw_e_dict.update({'translation': [float(el) for el in [tw_e.x, tw_e.y, tw_e.z]]})
    tw_e_dict.update({'rotation': [float(el) for el in [qw_e.w, qw_e.x, qw_e.y, qw_e.z]]})
    
    t0_w2_dict = dict()
    t0_w2_dict.update({'translation': [float(el) for el in [t_0_w2.x, t_0_w2.y, t_0_w2.z]]})
    t0_w2_dict.update({'rotation': [float(el) for el in [q_0_w2.w, q_0_w2.x, q_0_w2.y, q_0_w2.z]]})

    tw_e2_dict = dict()
    tw_e2_dict.update({'translation': [float(el) for el in [t_w_e2.x, t_w_e2.y, t_w_e2.z]]})
    tw_e2_dict.update({'rotation': [float(el) for el in [q_w_e2.w, q_w_e2.x, q_w_e2.y, q_w_e2.z]]})
    
    
    tsr_dict = {'manipind': 0,
                'relativebodyname': 'NULL',
                'relativelinkname': 'NULL',
                'T0_w': t0_w_dict, 
                'Tw_e': tw_e_dict,
                'Tw_e2': tw_e2_dict,
                'T0_w2': t0_w2_dict,
                'Bw': [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [-1.58, 0]],
                'sample_goal_from_chain': False,
                'sample_start_from_chain': False,
                'constrain_to_chain': True,
                'mimicbodyname': 'NULL'}
    
    config_data.update({'tsr': tsr_dict})

    with open(os.path.join(doors_tsr_configs_path, config_filename), 'w') as f:
        yaml.dump(config_data, f,  default_flow_style=None)

