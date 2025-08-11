#!/usr/bin/python

import rospy
import numpy as np

import sys
from core.util import read_config
from core.transforms import rot_z
import csv
from gazebo_push_open.cabinet_model import Cabinet
import time
import tqdm
rospy.init_node('node_generate_poses')


# cfg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/simulations_real_robot_config.yaml'
cfg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/simulations_exp3_axis_left.yaml'
save_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/door_params_poses_exp3_axis_left.npy'
config = read_config(cfg_path)

np.random.seed(69)



exp_name = 'exp3' # exp1, exp2, exp3, exp_real

base_size = 0.3
far_dist_range = 0.9

n = 1000
poses = np.zeros((n, 8))
cabinet_door_dims = config['cabinet_door_dims'] # w_door, h_door, static_d, d_door
cabinet_pose = config['cabinet_pose']

T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

# poses[:, 0] = np.random.uniform(cabinet_door_dims['min_width'], cabinet_door_dims['max_width'], size=n)
# poses[:, 1] = np.random.uniform(cabinet_door_dims['min_height'], cabinet_door_dims['max_height'], size=n)
# poses[:, 2] = np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x'], size=n)
# poses[:, 3] =  np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y'], size=n)
# poses[:, 4] = poses[:, 1]*0.5+0.009
# poses[:, 5] = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'], size=n)
# print(poses)

i = 0
start = time.time()
pbar = tqdm.tqdm(total=n)
while i < n: 
    door_params = [np.random.uniform(cabinet_door_dims['min_width'], cabinet_door_dims['max_width']),
                np.random.uniform(cabinet_door_dims['min_height'], cabinet_door_dims['max_height']),
                cabinet_door_dims['depth'],
                cabinet_door_dims['static_depth']]

    cabinet_position = [np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x']),
                        np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y']),
                        door_params[1]/2 + 0.009] # half of door height + offset from bottom
    rotz_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])

    axis_pos = cabinet_pose['axis_pos']
    # axis_pos = np.random.randint(0, 1) * 2 - 1

    if exp_name == 'exp_real':
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(cabinet_position)
        # T_A_S[2, 3] += 0.009
        Tz_init = np.eye(4)
        Tz_init[:3, :3] = rot_z(np.radians(90.))
        T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rotz_deg))
        T_A_S = T_A_S @ Tz
        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array(door_params), 
                                axis_pos=cabinet_pose['axis_pos'],
                                T_A_S=T_A_S,
                                save_path=config['cabinet_urdf_save_path'])
        
        T_pt1_A = np.eye(4)
        T_pt1_A[:3, 3] = np.array([0., cabinet_model.axis_distance, 0.])
        T_pt2_A = np.eye(4)
        T_pt2_A[:3, 3] = np.array([cabinet_model.static_d, cabinet_model.axis_distance, 0.])
        T_pt3_A = np.eye(4)
        T_pt3_A[:3, 3] = np.array([cabinet_model.static_d, cabinet_model.axis_distance - cabinet_model.w_door, 0.])
        T_pt4_A = np.eye(4)
        T_pt4_A[:3, 3] = np.array([0., cabinet_model.axis_distance - cabinet_model.w_door, 0.])

        T_pt1_S = T_A_S @ T_pt1_A
        T_pt2_S = T_A_S @ T_pt2_A
        T_pt3_S = T_A_S @ T_pt3_A
        T_pt4_S = T_A_S @ T_pt4_A

        if T_pt1_S[0, 3] > -0.4 and T_pt2_S[0, 3] > -0.4 and T_pt3_S[0, 3] < 0.4 and T_pt4_S[0, 3] < 0.4:
            poses[i, :] = np.array([door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg, axis_pos])
            # print(poses[i])
            i += 1
    
    elif exp_name == 'exp3':
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(cabinet_position)
        T_A_S[2, 3] += T_B_S[2, 3]
        Tz_init = np.eye(4)
        Tz_init[:3, :3] = rot_z(np.radians(90.))
        # T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rotz_deg))
        T_A_S = T_A_S @ Tz
        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array(door_params), 
                                axis_pos=cabinet_pose['axis_pos'],
                                T_A_S=T_A_S,
                                save_path=config['cabinet_urdf_save_path'])
        
        
        
        
        angle_deg = axis_pos * np.rad2deg(np.arctan(0.055/door_params[0]))

        
        T_D_S_0 = cabinet_model.T_A_S @ cabinet_model.T_D_A_init
        Tz_45 = np.eye(4)
        Tz_45[:3,:3] = rot_z(np.radians(axis_pos * 45.))
        T_D_S_45 = cabinet_model.T_A_S @ Tz_45 @ cabinet_model.T_D_A_init 
        Tz_90 = np.eye(4)
        Tz_90[:3,:3] = rot_z(np.radians(axis_pos * 90.))
        T_D_S_90 = cabinet_model.T_A_S @ Tz_90 @ cabinet_model.T_D_A_init
        Tz_state_angle = np.eye(4)
        Tz_state_angle[:3,:3] = rot_z(np.radians(angle_deg))
        T_D_S_angle =cabinet_model.T_A_S @ Tz_state_angle @ cabinet_model.T_D_A_init

        dist_0 = np.linalg.norm(T_D_S_0[:2, 3])
        dist_45 = np.linalg.norm(T_D_S_45[:2, 3])
        dist_90 = np.linalg.norm(T_D_S_90[:2, 3])
        dist_state_angle = np.linalg.norm(T_D_S_angle[:2, 3])
        dist_axis = np.linalg.norm(T_A_S[:2, 3])

        # close_dist_range = base_size + door_params[0]
        close_dist_range = base_size
        dist_crit = close_dist_range < dist_0 < far_dist_range and close_dist_range < dist_45 < far_dist_range and close_dist_range < dist_90 < far_dist_range and close_dist_range < dist_state_angle < far_dist_range 
        
        dist_axis = np.linalg.norm(T_A_S[:2, 3])
        close_axis_dist_from_base = dist_axis > base_size 

        is_properly_rotated = np.dot(T_D_S_0[:3, 3], T_D_S_0[:3, 2]) > 0.


        # if 0.3 < dist_axis < 0.5 and dist_crit:
        #     print(dist_crit, close_axis_dist_from_base, is_properly_rotated, T_D_S_0[1, 3] > 0.)
        #     print(np.array([door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg, angle_deg, axis_pos]))
        #     print('\n')

        # if dist_crit and close_axis_dist_from_base and is_properly_rotated :
        if dist_crit and close_axis_dist_from_base and is_properly_rotated and T_D_S_0[1, 3] > 0.:
        # if dist_crit and is_properly_rotated and T_D_S_0[1, 3] > 0.:
            poses[i, :] = np.array([door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg, angle_deg, axis_pos])
            i += 1
            pbar.update(1)

print(time.time() - start)
pbar.close()
np.save(save_path, poses)

rospy.signal_shutdown('shutdown')