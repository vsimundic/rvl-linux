import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

poses = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/door_params_poses_exp3_axis_left.npy')
poses = poses[:, 2:5]

plt.figure()
plt.scatter(poses[:,0], poses[:,1])
plt.show()



read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_exp3.csv'
data = df = pd.read_csv(filepath_or_buffer=read_results_path, sep=',', header=0)
 

# data = data.loc[((data['path_found'] == True) & 
#                         (data['traj_success'] == False) & 
#                         (data['contact_free'] == False) & 
#                         (data['door_opened'] == False))] 
data = data.loc[data['path_found'] == True]

x = data.loc[:, 'x'].to_numpy()
y = data.loc[:, 'y'].to_numpy()
z = data.loc[:, 'z'].to_numpy()
plt.figure()
plt.scatter(y, x)
plt.show()

