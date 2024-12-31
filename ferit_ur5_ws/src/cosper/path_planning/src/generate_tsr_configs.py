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
doors_tsr_configs_path = os.path.join(pkg_path, 'tsr_data', 'cabinet_configs')
urdf_path = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.urdf'

ADD_TO_EXISTING_CFGS = True


if not os.path.exists(doors_tsr_configs_path):
    os.makedirs(doors_tsr_configs_path)


T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

if not ADD_TO_EXISTING_CFGS:
    robot = UR5Commander()
else:
    robot = None

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
    Tz[:3, :3] = rot_z(np.radians(door[ 5]))
    T_A_S = T_A_S @ Tz
    axis_pos = door[-1]
    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array([door[0], door[1], 0.018, 0.4]), 
                            axis_pos=axis_pos,
                            T_A_S=T_A_S,
                            save_path=urdf_path,
                            has_handle=True)
    cabinet_mesh_filename = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.ply'



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
    Tw_e[:3, :3] = np.array([[0, 0, -axis_pos],
                                [axis_pos, 0, 0],
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
    T_T_B_goal = np.linalg.inv(robot.T_B_S) @ cabinet_model.T_A_S @ T_T_A

    q_goal = robot.get_inverse_kin_builtin(robot.get_current_joint_values(), T_T_B_goal)
    if q_goal is None:
        rospy.logwarn('Initial config not found. Setting default.')
        q_goal = [0.0, float(np.deg2rad(-90.)), 0.0, float(np.deg2rad(-90.)), 0.0, 0.0]

    # print("initial_config: {}".format(q_goal))

    Tz_90 = np.eye(4) 
    Tz_90[:3,:3] = rot_z(np.deg2rad(-90))
    T_A_S_rotated = T_A_S @ Tz_90
    T_T_B_goal_rotated = np.linalg.inv(robot.T_B_S) @ T_A_S_rotated @ T_T_A

    q_goal_rot = robot.get_inverse_kin_builtin(q_goal, T_T_B_goal_rotated)
    if q_goal_rot is None:
        # print("Cannot rotate doors! Aborting.")
        rospy.logwarn('Goal config not found. Setting default.')
        q_goal_rot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # print("goal_config: {}".format(q_goal_rot))



    config_data = dict()
    config_data.update({'scene_file': '/home/user/tsr_ws/ur5.dae'})
    config_data.update({'ground_file': '/home/user/tsr_ws/ground.xml'})
    config_data.update({'robot_name': 'ur5_robotiq_ft_3f'})
    config_data.update({'dof': 6})
    config_data.update({'initial_config': q_goal})
    config_data.update({'goal_config': q_goal_rot})
    config_data.update({'psample': 0.0})
    
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
                'Bw': [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [-1.58, 0]],
                'sample_goal_from_chain': False,
                'sample_start_from_chain': False,
                'constrain_to_chain': True,
                'mimicbodyname': 'NULL'}
    
    config_data.update({'tsr': tsr_dict})



    # print(tsr_dict)
    with open(os.path.join(doors_tsr_configs_path, config_filename), 'w') as f:
        yaml.dump(config_data, f,  default_flow_style=None)

