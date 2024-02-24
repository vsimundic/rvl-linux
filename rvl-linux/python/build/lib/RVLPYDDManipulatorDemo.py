import numpy as np
import rvlpyutil as rvlutil
import os

import RVLPYDDManipulator as rvl

print('Starting...')

manipulator = rvl.PYDDManipulator()
manipulator.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec.cfg')
manipulator.load_tool_model('/home/RVLuser/rvl-linux/modules/RVLMotionDemo/Robotiq3Finger')
manipulator.load_feasible_tool_contact_poses('/home/RVLuser/rvl-linux/data/DDMan/3finger_gripper/feasible_poses_axis_left.npy')
T_0_S = np.eye(4)
T_0_S[2,3] = 0.005
manipulator.set_robot_pose(T_0_S)
# T_F_S = np.array([[0.0, 0.0, 1.0, 0.6], [-1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.546], [0, 0, 0, 1]])
# manipulator.set_furniture_pose(T_F_S)
manipulator.set_door_model_params(0.018, 0.3, 0.5, 0.0, -0.14, -1.0, 0.018, 0.005)
T_A_S = np.array([[-1.0, 0.0, 0.0, -0.5], [0.0, -1.0, 0.0, -0.6], [0.0, 0.0, 1.0, 0.278], [0, 0, 0, 1]])
manipulator.set_door_pose(T_A_S)
# R_G_S_Y = rvlutil.roty(0.75 * np.pi)
# R_G_S_Z = rvlutil.rotz(np.pi)
# t_0_S_home = np.array([0.3 + 0.0823, 0.0, 0.5])
# T_G_S_init = R_G_S_Y @ R_G_S_Z
# T_G_S_init[:3,3] = t_0_S_home
# print(T_G_S_init)
q_init = np.deg2rad(np.array([166.999724, -62.99970028, -58.00079828, -95.99618458, 110.00280307, 65.99985772]))
manipulator.set_environment_state(-12.0)
print('Path planning...')
T_G_0, q = manipulator.path2(q_init)
print('completed.')
if T_G_0.shape[0] == 1:
    print('Path is not found!')
# manipulator.set_environment_state(0.0)
# T_DD_S = manipulator.get_T_DD_S()
# print('T_DD_S')
# print(T_DD_S)
# T_F_S = manipulator.get_T_F_S()
# print('T_F_S')
# print(T_F_S)
# T_A_F = np.array([[0, 1, 0, 0.018+0.005+0.15+0.14], [0, 0, -1, 0.018+0.005+0.25], [-1, 0, 0, 0.009], [0, 0, 0, 1]])
# T_F_S_ = T_A_S @ np.linalg.inv(T_A_F)
# T_DD_A = manipulator.get_T_DD_A()
# T_DD_S_ = T_A_S @ T_DD_A
# print('end.')
# for i in range(T_G_0.shape[0]):
#     q_, success = manipulator.inv_kinematics(T_G_0[i,:,:].astype('double'))
#     T_G_0_ = manipulator.fwd_kinematics(q_.astype('double'))
#     print('.')


