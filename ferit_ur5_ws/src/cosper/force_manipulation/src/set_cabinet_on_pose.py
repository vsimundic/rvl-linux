import rospy
import numpy as np
import os
import pickle   

base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'

door_thickness = 0.018
width = 0.395
height = 0.495

IS_OFFLINE = False
cabinet_idx = 9

# num_poses = 3 if IS_OFFLINE else 1

T_A_W_path = os.path.join(base_dir, 'gt_cabinets', 'offline_cabinet_gt_poses.pkl' if IS_OFFLINE else 'cabinet_gt_poses.pkl')
with open(T_A_W_path, 'rb') as f:
    T_A_W_cabinets = pickle.load(f)

T_A_W = T_A_W_cabinets[cabinet_idx][0]

T_corner1_A = np.eye(4)
T_corner1_A[:3, 3] = np.array([-door_thickness, 0.0, height*0.5])

T_corner2_A = np.eye(4)
T_corner2_A[:3, 3] = np.array([-door_thickness, -width, height*0.5])

T_corner1_W = T_A_W @ T_corner1_A
T_corner1_W[:3, 3] *= 1000. # convert to mm
T_corner2_W = T_A_W @ T_corner2_A
T_corner2_W[:3, 3] *= 1000. # convert to mm

print('Pose %d - Topleft corner: %.2f %.2f %.2f' % (cabinet_idx, -T_corner1_W[0, 3], -T_corner1_W[1, 3], T_corner1_W[2, 3]))
print('Pose %d - Topright corner: %.2f %.2f %.2f' % (cabinet_idx, -T_corner2_W[0, 3], -T_corner2_W[1, 3], T_corner2_W[2, 3]))
print('')