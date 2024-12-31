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
import core.meshes as fc_meshes # ferit_core_meshes

np.random.seed(69)

doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/door_configurations_axis_left.npy')

pkg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning'
doors_tsr_configs_path = os.path.join(pkg_path, 'tsr_data', 'cabinet_configs')
urdf_path = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.urdf'

T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

for i_door in tqdm(range(doors.shape[0])):
# for i_door in range(2):

    door = doors[i_door]

    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array(door[2:5])
    T_A_S[2, 3] += T_B_S[2, 3]
    # T_A_S[1, 3] += 0.1

    # Tz_init = np.eye(4)
    # Tz_init[:3, :3] = rot_z(np.radians(90.))
    # T_A_S = T_A_S @ Tz_init
    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.radians(door[5]))
    T_A_S = T_A_S @ Tz
    axis_pos = door[-1]
    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array([door[0], door[1], 0.018, 0.4]),
                            axis_pos=axis_pos,
                            T_A_S=T_A_S,
                            save_path=urdf_path,
                            has_handle=True)
    # cabinet_mesh_filename = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.ply'
    
    mesh_file = os.path.join(doors_tsr_configs_path, 'cabinet_%d.ply' % i_door)
    mesh_file_dae = os.path.join(doors_tsr_configs_path, 'cabinet_%d.dae' % i_door)
    cabinet_name = 'cabinet'
    cabinet_model.save_mesh(mesh_file)

    fc_meshes.convert_to_dae(mesh_file, mesh_file_dae, cabinet_name)

    config_filename = 'cabinet_%d.yaml' % i_door


    ### TSR matrices definition ###
    T0_w = T_A_S
    T0_w_pose =  matrix_to_pose(T0_w)
    t0_w = T0_w_pose.position
    q0_w = T0_w_pose.orientation

    # print("  T0_w:")
    # print('    translation: [%f, %f, %f]' % (t0_w.x, t0_w.y, t0_w.z))
    # print('    rotation: [%f, %f, %f, %f] #wxyz' % (q0_w.w, q0_w.x, q0_w.y, q0_w.z))

    T_6_H = np.eye(4)
    T_6_H[:3, :3] = np.array([[0, 0, -axis_pos],
                                [axis_pos, 0, 0],
                                [0, -1, 0]])

    Tw_e = cabinet_model.T_H_A.copy()
    Tw_e[:3, :3] = np.array([[0, 0, 1],
                            [-1, 0, 0],
                            [0, -1, 0]])
    Tw_e[0, 3] -= 0.28
    Tz = np.eye(4)
    Tz[:3,:3] = rot_z(np.radians(45.))
    Tw_e = Tw_e @ Tz
    Tw_e_pose =  matrix_to_pose(Tw_e)
    tw_e = Tw_e_pose.position
    qw_e = Tw_e_pose.orientation
    # print('  Tw_e:')
    # print('    translation: [%f, %f, %f]' % (tw_e.x, tw_e.y, tw_e.z))
    # print('    rotation: [%f, %f, %f, %f] #wxyz' % (qw_e.w, qw_e.x, qw_e.y, qw_e.z))

    T_T_A = Tw_e


    T_O_S_pose = matrix_to_pose(cabinet_model.T_O_S)
    t_O_S = np.array([T_O_S_pose.position.x, T_O_S_pose.position.y, T_O_S_pose.position.z])
    q_O_S = np.array([T_O_S_pose.orientation.w, T_O_S_pose.orientation.x, T_O_S_pose.orientation.y, T_O_S_pose.orientation.z])

    file = open(os.path.join(doors_tsr_configs_path, config_filename), 'r')
    data = yaml.safe_load(file)
    file.close()

    T_O_S_dict = dict()
    T_O_S_dict.update({'translation': [float(el) for el in t_O_S.tolist()]})
    T_O_S_dict.update({'rotation': [float(el) for el in q_O_S.tolist()]})

    # data.update({'T_O_S': T_O_S_dict})
    # data.update({'cabinet_model_name': cabinet_name})

    file = open(os.path.join(doors_tsr_configs_path, config_filename), 'w')
    yaml.dump(data, file,  default_flow_style=None)
    file.close()
    pass