import pandas as pd
import numpy as np


def rot_z(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    
    return R

results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_exp1_final.csv'
results_path3 = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_exp3.csv'

data = pd.read_csv(filepath_or_buffer=results_path, sep=',', header=0)
data3 = pd.read_csv(filepath_or_buffer=results_path3, sep=',', header=0)

# data = data.loc[((data['path_found'] == False) & 
#                         (data['traj_success'] == False) & 
#                         (data['contact_free'] == True) & 
#                         (data['door_opened'] == False))] 
data = data.loc[(data['path_found'] == False)] 
data3 = data3.loc[(data3['path_found'] == False)] 

idxs1 = data.axes[0].values
idxs3 = data3.axes[0].values
diff = idxs1 - idxs3

T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

for i, row in data.iterrows():

    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array([row['x'], row['y'], row['z']])
    T_A_S[2, 3] += T_B_S[2, 3] # adjust with robot base

    Tz_init = np.eye(4)
    Tz_init[:3, :3] = rot_z(np.radians(90.))
    
    T_A_S = T_A_S @ Tz_init
    
    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.radians(row['rot_z']))
    T_A_S = T_A_S @ Tz
    print(T_A_S)