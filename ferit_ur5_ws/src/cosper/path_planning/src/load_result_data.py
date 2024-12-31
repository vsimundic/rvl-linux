#!/usr/bin/python

from core.util import read_config, read_csv_DataFrame
import numpy as np
from core.transforms import rot_z, pose_to_matrix
from gazebo_push_open.cabinet_model import Cabinet
import os
from tqdm import tqdm

lock_T_G_DD = False

read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results_single_contact_handle_moveit copy.csv'
data = read_csv_DataFrame(read_results_path)

flags = [[False, False, False],
            [False, False, True],
            [False, True, False],
            [False, True, True],
            [True, False, False],
            [True, False, True],
            [True, True, False],
            [True, True, True]]


# data = data.loc[((data['path_found'] == True) & 
#                         (data['traj_success'] == True) & 
#                         (data['contact_free'] == True) & 
#                         (data['door_opened'] == True))] 
successful_indices = data.loc[((data['path_found'] == True) & 
                        (data['traj_success'] == True) & 
                        (data['contact_free'] == True) & 
                        (data['door_opened'] == True))].index.to_list()
print(successful_indices)
doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/door_configurations_axis_left.npy')

T_R_W = np.eye(4)
T_R_W[2, 3] = 0.005

save_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/ikfast_successful_matrices/'


np.save(os.path.join(save_path, 'T0_B.npy'), T_R_W)


for idx in tqdm(successful_indices):
    # print(data.iloc[idx])
    
    door = doors[idx, :]
    width = door[0]
    height = door[1]
    state_angle = door[6]
    axis_pos = door[7]
    rot_z_deg = door[5]
    position = door[2:5]


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
                            save_path=None,
                            has_handle=True)

    T0_w = T_A_S

    Tw_e = cabinet_model.T_H_A.copy()
    Tw_e[:3, :3] = np.array([[0, 0, 1],
                            [-1, 0, 0],
                            [0, -1, 0]])
    Tw_e[0, 3] -= 0.28
    Tz = np.eye(4)
    Tz[:3,:3] = rot_z(np.radians(45.)) # rotate gripper 45 deg to align fingers with handle
    Tw_e = Tw_e @ Tz


    np.save(os.path.join(save_path, 'cabinet_%d_T0_w.npy' % idx), T0_w)
    np.save(os.path.join(save_path, 'cabinet_%d_Tw_e.npy' % idx), Tw_e)
