#!/usr/bin/python

import numpy as np
import RVLPYDDManipulator as rvlpy_dd_man
from timeit import default_timer as timer
import time
import pandas as pd
import os

def rot_z(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    
    return R

doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/door_params_poses_axis_left.npy')
T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005


csv_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_exp3.csv'
graph_path = '/home/RVLuser/rvl-linux/data/Robotiq3Finger/contact_pose_graph.dat'
df = pd.read_csv(filepath_or_buffer=csv_path, sep=',', header=0)
mask_successful = df['path_found'].to_numpy()
mask_unsuccessful = np.invert(mask_successful)

doors_successful = doors[mask_successful]
doors_unsuccessful = doors[mask_unsuccessful]

q_init = np.array([0., -1.5674883378518185, 0., -1.5676032728569234, 0., 0.])
# adjust joint values from ROS
q_init[0] += np.pi
q_init[5] += np.pi
q_init[q_init>np.pi]-=(2.0*np.pi)     
q_init[q_init<-np.pi]+=(2.0*np.pi)



n = 50
times = np.zeros((n,),dtype=np.float64)
# for i in range(doors.shape[0]):
for i in range(n):
    # door = doors[i, :]
    # door = doors_unsuccessful[i, :]
    door = doors_successful[i, :]
    print('\n')
    print(doors[i])
    print('\n')
    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array(door[2:5])
    T_A_S[2, 3] += T_B_S[2, 3]
    Tz_init = np.eye(4)
    Tz_init[:3, :3] = rot_z(np.radians(90.))
    # T_A_S = T_A_S @ Tz_init
    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.radians(door[ 5]))
    T_A_S = T_A_S @ Tz

    # if os.path.exists(graph_path):
    #     os.remove(graph_path)
    # else:
    #     print("The file does not exist")

    # Path planning setup
    start = timer()
    path_planner = rvlpy_dd_man.PYDDManipulator()
    path_planner.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec.cfg')
    path_planner.set_robot_pose(T_B_S)
    path_planner.set_door_model_params(
                                    0.018,
                                    door[0],
                                    door[1],
                                    0.0, # rx
                                    -(door[0]/2. - 0.01), # ry
                                    -1.0, # opening direction
                                    0.0018,
                                    0.005)
    path_planner.set_door_pose(T_A_S)
    path_planner.set_environment_state(door[6])

    T_G_0_array, q = path_planner.path2(np.array(q_init), -90.0, 17, False)
    diff = timer() - start 
    times[i] = diff



    del path_planner

print('Min: %.4f' % times.min())
print('Max: %.4f' % times.max())
print('Avg: %.4f' % times.mean())