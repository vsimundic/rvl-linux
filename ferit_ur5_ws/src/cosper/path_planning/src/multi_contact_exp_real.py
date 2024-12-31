#!/usr/bin/python

import rospy
import os
import rospkg

from core.util import read_config
from core.ur5_commander import UR5Commander
from core.rvl import RVLRGBD2PLY
from gazebo_push_open.cabinet_model import Cabinet
from DDMan import push
import RVLPYDDManipulator as rvlpy_dd_man
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
from core.transforms import rot_z, pose_to_matrix
import RVLPYDDDetector as rvl_dddetector
from trac_ik_python.trac_ik import IK
import yaml
import open3d as o3d
import json
from geometry_msgs.msg import Pose
from rospkg import RosPack

if __name__ == '__main__':
	rospy.init_node('node_test_real_cabinet')

	rp = RosPack()
	pkg_path = rp.get_path('path_planning')


	save_path = os.path.join(pkg_path, 'config/real_robot/Exp-real_robot_cabinet_open')
	if not os.path.isdir(save_path):
		os.makedirs(save_path)

	# Robot handler
	robot = UR5Commander()

	T_G_0_vertices = np.zeros((2, 4, 4), dtype=np.float32)
	T_G_T = np.load(os.path.join(pkg_path, 'config/T_G_T_pen.npy'))
	T_G_T[:3, 3] = np.array([0., 0., 0.310])

	T_RP_TCP = np.eye(4)
	T_RP_TCP[:3, 3] = np.array([0.0775, 0., 0.097])
	T_TCP_T = np.eye(4)
	T_TCP_T[:3, :3] = rot_z(np.deg2rad(-45.0))
	T_TCP_T[2, 3] = 0.175
	T_RP_T = T_TCP_T @ T_RP_TCP


	T_T_0_new = robot.get_current_tool_pose()
	T_RP_0 = T_T_0_new @ T_RP_T

	T_T_0_ = robot.get_current_tool_pose()

	T_G_0 = T_T_0_ @ T_G_T


	T_T_0_new = T_G_0 @ np.linalg.inv(T_G_T)

	robot.send_pose_to_robot(T_T_0_new)

	print(T_T_0_[:3,3] - T_T_0_new[:3,3])


	i = 0
	while i < 2:
		key = input('Press \'p\' to save tool pose: ')
		if key == 'p':
			T_T_0_ = robot.get_current_tool_pose()
			T_G_0_vertices[i, :, :] = T_T_0_ @ T_G_T
			np.save(save_path + 'T_T_0_v%d.npy' % i, T_T_0_)
			i += 1

	T_G_0_vertices[0, :, :] = np.load(save_path + '/T_T_0_v0.npy') @ T_G_T
	T_G_0_vertices[1, :, :] = np.load(save_path + '/T_T_0_v1.npy') @ T_G_T

	#  t_G_0_0 = T_G_0_vertices[0, :3, 3].copy() + np.array([0., 0., 0.015])
	#  t_G_0_1 = T_G_0_vertices[1, :3, 3].copy() + np.array([0., 0., 0.015])
	t_G_0_0 = T_G_0_vertices[0, :3, 3].copy()
	t_G_0_1 = T_G_0_vertices[1, :3, 3].copy()

	t_G_0_1_0 = np.array([t_G_0_1[0], t_G_0_1[1], t_G_0_0[2]]) # point that has same x, y as second point, but is on the same height as first


	t_G_0_y = t_G_0_0 - t_G_0_1
	t_G_0_y /= np.linalg.norm(t_G_0_y)

	z_axis = np.array([0., 0., 1.])

	# x_axis = np.cross(z_axis, t_G_0_y)
	x_axis = np.cross(t_G_0_y, z_axis)

	R_A_0 = np.array([x_axis, 
						t_G_0_y, 
						z_axis])
	R_A_0 = R_A_0.T
	T_A_0_ = np.eye(4)
	T_A_0_[:3, :3] = R_A_0
	T_A_0_[:3, 3] = t_G_0_0

	T_Aa_A = np.eye(4)
	# z_ = (t_G_0_0[2]-0.025)/2.
	z_ = (t_G_0_0[2])/2.
	T_Aa_A[:3, 3] = np.array([0.018/2., 0., -z_])

	T_A_0 = T_A_0_ @ T_Aa_A

	np.save(save_path + '/T_A_0.npy', T_A_0)

	print(T_A_0)
	# np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_A_0.npy', T_A_0)
	cabinet_depth = 0.018
	cabinet_width = 0.396
	cabinet_height = 0.496


	T_D_A = np.eye(4)
	T_D_A[:3, 3] = np.array([0.009, -cabinet_width, cabinet_height/2.])

	T_D_0 = T_A_0 @ T_D_A


	# Push mechanism pts
	T_G_T_closed_gripper = np.eye(4)
	T_G_T_closed_gripper[2, 3] = 0.28

	T_P_A = np.array([[0, 0, 1, -cabinet_depth*0.5],
						[0, 1, 0, -(cabinet_width-0.04)],
						[-1, 0, 0, cabinet_height*0.5-0.04],
						[0, 0, 0, 1]], dtype=np.float32)

	T_P0_A = T_P_A.copy()
	T_P0_A[0, 3] -= 0.1

	T_P1_A = T_P_A.copy()
	T_P1_A[0, 3] += 0.1

	T_Tpt0_0 = T_A_0 @ T_P0_A @ np.linalg.inv(T_G_T_closed_gripper)
	T_Tpt_0 = T_A_0 @ T_P_A @ np.linalg.inv(T_G_T_closed_gripper)
	T_Tpt1_0 = T_A_0 @ T_P1_A @ np.linalg.inv(T_G_T_closed_gripper)

	q_init = robot.get_current_joint_values()

	# np.save(save_path + 'q_init.npy', np.array(q_init))

	# q0 = robot.get_inverse_kin(q_init, T_Tpt0_0)
	# q1 = robot.get_inverse_kin(q0, T_Tpt_0)
	# # q2 = robot.get_inverse_kin(q1, T_Tpt1_0)
	# q0 = np.array(q0)
	# # q2 = np.array(q2)
	# q_ = np.array([q0, q1, q0, np.array(q_init)])
	# robot.open_gripper_pinch()

	# robot.close_gripper_pinch()

	# robot.generate_URScript(q_, with_force_mode=False)
	# # robot.zero_ft_sensor()
	# robot.send_URScript()

	# rospy.sleep(30)

	# robot.open_gripper_pinch()
	# rospy.sleep(1)

	config_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/config/config_real_cabinet.yaml'
	config = read_config(config_path)
	feasible_poses_args = config['feasible_poses']
	feasible_poses_args['door_dims'] = np.array([cabinet_width, cabinet_height, cabinet_depth])
	feasible_poses_args['static_depth'] = 0.3
	feasible_poses_args['opening_direction'] = -1.0
	feasible_poses = push.demo_push_poses_ros(**feasible_poses_args)

	path_planner = rvlpy_dd_man.PYDDManipulator()
	path_planner.create(config['rvl_config_path'])
	path_planner.load_tool_model(config['tool_model_params'])
	path_planner.set_environment_state(-7.5)
	path_planner.load_feasible_tool_contact_poses(feasible_poses_args['feasible_poses_path'])
	path_planner.set_robot_pose(np.eye(4))
	path_planner.set_door_model_params(
									cabinet_depth,
									cabinet_width,
									cabinet_height,
									0.0, # rx
									-cabinet_width*0.5, # ry
									-1.0, # opening direction
									0.018,
									0.005) 
	path_planner.set_door_pose(T_A_0)


	q_init = robot.get_current_joint_values()
	q_init = np.array(q_init)

	q_init[0] += np.pi
	q_init[5] += np.pi
	q_init[q_init>np.pi]-=(2.0*np.pi)     
	q_init[q_init<-np.pi]+=(2.0*np.pi)
	# q_init[0, :] = robot.joint_values_init

	# T_G_0_array, q = path_planner.path2(np.array(q_init))
	T_G_0_array, q = path_planner.path2(np.array(q_init), -90.0, 17, False)

	if T_G_0_array.shape[0] == 1:
		print('Path is not found!')
		exit()

	# T_PR_G_ = np.eye(4)
	# T_PR_G_[:3, 3] = np.array([0.0775, 0., 0.102])

	# T_G_0_ = T_G_0_vertices[1]
	# # T_T_0_ = T_G_0_ @ np.linalg.inv(T_G_T)
	# # T_G_0_grip = np.eye(4)
	# # T_G_0_grip[2, 3] = 0.115

		


	# T_0_D = np.linalg.inv(T_D_0)
	# T_PR_0_array = T_G_0_array @ T_PR_G_[np.newaxis, ...]

	# for i in range(T_PR_0_array.shape[0]):
	#     T_PR_0_ =T_PR_0_array[i]
	#     print(np.linalg.inv(T_PR_0_) @ T_G_0_)

	# # print(np.linalg.inv(T_PR_0_array[:]) @ T_D_0[np.newaxis, ...])


	q[:, 0] += np.pi
	q[:, 5] += np.pi
	q[q>np.pi]-=(2.0*np.pi)     
	q[q<-np.pi]+=(2.0*np.pi)
	# q[0, :] = q_init

	# np.save(save_path +'q.npy', q)
	# q = np.load(save_path +'q.npy')


	robot.generate_URScript(q, with_force_mode=False)
	# robot.generate_URScript_poses(T_G_0_array)
	robot.send_URScript()

	# # Compare SA to marker 4
	# with open('/home/RVLuser/ferit_ur5_ws/src/other/calibration_program/data/markers/marker%d.json' % 4) as f:
	#     data = json.load(f)
	#     pose = Pose()
	#     pose.position.x = data["position"]["x"]
	#     pose.position.y = data["position"]["y"]
	#     pose.position.z = data["position"]["z"]
	#     pose.orientation.x = data["orientation"]["x"]
	#     pose.orientation.y = data["orientation"]["y"]
	#     pose.orientation.z = data["orientation"]["z"]
	#     pose.orientation.w = data["orientation"]["w"]

	#     T_T4_S = pose_to_matrix(pose)
	#     T_M4_S = T_T4_S @ T_G_T
	#     print("Loaded marker {} from file:".format(4))


	# dif = T_A_0 - T_M4_S
	# print(dif)


