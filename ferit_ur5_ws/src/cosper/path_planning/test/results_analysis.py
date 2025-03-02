import numpy as np
import pandas as pd

read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results/results_multi-c_our_handleless.csv'
# read_results_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results_simulation_tsr_single_contact_final.csv'
data = pd.read_csv(read_results_path, sep=',', header=0)

flags = [[True, False, False, False],
        [True, False, False, True],
        [True, False, True, False],
        [True, False, True, True],
        [True, True, False, False],
        [True, True, False, True],
        [True, True, True, False],
        [True, True, True, True]]

data_ = data.loc[((data['path_found'] == False) & 
                (data['traj_success'] == False) & 
                (data['contact_free'] == True) & 
                (data['door_opened'] == False))] 

print(str([False, False, True, False]) + ': %d' %data_.shape[0])


for i in range(len(flags)):
    flags_ = flags[i]

    data_ = data.loc[((data['path_found'] == flags_[0]) & 
                    (data['traj_success'] == flags_[1]) & 
                    (data['contact_free'] == flags_[2]) & 
                    (data['door_opened'] == flags_[3]))] 

    print(str(flags_) + ': %d' %data_.shape[0])