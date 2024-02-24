#!/usr/bin/python

import rospy
import os
import rospkg
from core.read_config import read_config
from core.paths_packages import get_package_name_from_node, get_package_path_from_name
from core.ur5_commander import UR5Commander
from DDMan import push
from gazebo_push_open.cabinet_model import Cabinet
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
from core.transforms import rot_z, matrix_to_pose
import RVLPYDDManipulator as rvlpy_dd_man
from trac_ik_python.trac_ik import IK

if __name__ == '__main__':
    rospy.init_node('node_path_planning')
    
    try:
        cfg_file = rospy.get_param('config_file')
    except rospy.exceptions.ROSException:
        raise Exception('Could not fetch param.')
    
    config = read_config(cfg_file)
    
    # Robot handler
    robot = UR5Commander()



    #### CABINET ####
    cabinet_door_dims = config['cabinet_door_dims'] # w_door, h_door, static_d, d_door 
    door_params = [np.random.uniform(cabinet_door_dims['min_width'], cabinet_door_dims['max_width']), 
                  np.random.uniform(cabinet_door_dims['min_height'], cabinet_door_dims['max_height']),
                  cabinet_door_dims['depth'],
                  cabinet_door_dims['static_depth']]
    
    cabinet_pose = config['cabinet_pose']
    feasible_poses_args = config['feasible_poses']

    # Cabinet pose in world
    cabinet_position = [np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x']),
                        np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y']),
                        door_params[1]/2 + 0.018 + 0.01 + 0.02] # half of door height + depth of bottom panel + double static moving part distance
    rotz_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])

    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array(cabinet_position)
    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.radians(rotz_deg))
    T_A_S = T_A_S @ Tz


    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array(door_params), 
                            axis_pos=cabinet_pose['axis_pos'],
                            T_A_S=T_A_S,
                            # position=np.array(cabinet_position),
                            # rotz_deg=rotz_deg,
                            save_path=config['cabinet_urdf_save_path'])
    
    # Save cabinet mesh to a file
    cabinet_model.save_mesh_without_doors(config['cabinet_mesh_save_path'])

    # Spawning model in Gazebo
    cabinet_model.delete_model_gazebo()
    cabinet_model.spawn_model_gazebo()
    T_D_S_gazebo = cabinet_model.get_door_panel_RF_wrt_world()

    # Open doors in Gazebo
    theta_deg = config['feasible_poses']['dd_state_deg']
    cabinet_model.open_door_gazebo(theta_deg)
    cabinet_model.change_door_angle(theta_deg)
    cabinet_model.update_mesh()

    # # TEST SPHERES
    # cabinet_model.delete_model_gazebo_sphere()
    # cabinet_model.spawn_model_gazebo_sphere()

    #### FEASIBLE POSES ####
    # Get feasible poses
    feasible_poses_args['door_dims'] = np.array(door_params[:3])
    feasible_poses_args['static_depth'] = door_params[3]
    feasible_poses = push.demo_push_poses_ros(**feasible_poses_args)


    # Path planning
    path_planner = rvlpy_dd_man.PYDDManipulator()
    path_planner.create(config['rvl_config_path'])
    path_planner.load_tool_model(config['tool_model_params'])
    path_planner.set_environment_state(config['feasible_poses']['dd_state_deg'])
    # path_planner.load_feasible_tool_contact_poses(feasible_poses_args['feasible_poses_path'])
    path_planner.load_feasible_tool_contact_poses(feasible_poses_args['feasible_poses_path'])
    path_planner.set_robot_pose(robot.T_B_S)
    # T_A_S = cabinet_model.T_O_S @ cabinet_model.T_A_O_init

    np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_A_S_temp.npy', T_A_S)
    path_planner.set_door_model_params(
                                       # T_A_S,
                                       cabinet_model.d_door,
                                       cabinet_model.w_door,
                                       cabinet_model.h_door,
                                       0.0, # rx
                                       -(cabinet_model.w_door/2. - cabinet_model.axis_distance), # ry
                                       -1.0, # opening direction
                                       cabinet_model.static_side_width,
                                       cabinet_model.moving_to_static_part_distance) 
    path_planner.set_door_pose(T_A_S)
    
    # T_G_S_init = robot.T_B_S @ robot.T_T_B_home @ robot.T_G_T
    # np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_G_S_init_temp.npy', T_G_S_init)
    
    # Get points for the whole path
    # T_D_S_gazebo = cabinet_model.get_door_panel_RF_wrt_world()
    # T_G_0, joint_values_list = path_planner.path2(T_G_S_init)

    # rospy.sleep(1.5)
    # robot.send_named_pose('up', wait=True)
    q_init = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/q_init.npy')
    # q_init = robot.get_current_joint_values()
    # np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/q_init.npy', np.array(q_init))



    robot.send_joint_values_to_robot(q_init.tolist())
    # q_init = np.array(q_init)

    q_init[0] += np.pi
    q_init[5] += np.pi
    q_init[q_init>np.pi]-=(2.0*np.pi)     
    q_init[q_init<-np.pi]+=(2.0*np.pi)
    # q_init[0, :] = robot.joint_values_init
    
    print(np.rad2deg(q_init[:]))

    T_G_0_array, q = path_planner.path2(np.array(q_init))


    """
    # path_planner.set_environment_state(config['feasible_poses']['dd_state_deg'])
    # T_D_S_rvl = path_planner.get_T_DD_S()
    # T_F_S_gazebo = cabinet_model.T_O_S @ cabinet_model.T_F_O
    # T_F_S_rvl = path_planner.get_T_F_S()
    # path_planner.set_environment_state(0.)
    # T_D_S__rvl0 = path_planner.get_T_DD_S()


    # # Add cabinet mesh to rviz
    # robot.add_mesh_to_scene(config['cabinet_mesh_save_path'], 'cabinet', np.linalg.inv(robot.T_B_S) @ cabinet_model.T_O_S)
    
    # Go to home pose
    # robot.send_pose_to_robot(robot.T_T_B_home)
    # robot.send_joint_values_to_robot(robot.joint_values_init)

    # robot.send_named_pose('up')

    # for i in range(T_G_0.shape[0]):
    #     rospy.loginfo('Pose %s' %i)
    #     T_G_S_ = T_G_0[i]
    #     T_T_B_ = robot.get_tool_pose_from_gripper_pose(T_G_S_)
    #     robot.send_pose_to_robot(T_T_B_)

    #     T_T_B_curr = robot.get_current_tool_pose()
    #     T_G_S_curr = robot.T_B_S @ T_T_B_curr @ robot.T_G_T

    # T_T_B_arr = []
    # for i in range(T_G_0.shape[0]):
    # # for i in range(3):
    #     rospy.loginfo('Pose %s' %i)
    #     T_G_S_ = T_G_0[i]
    #     T_T_B_ = robot.get_tool_pose_from_gripper_pose(T_G_S_)

    #     T_T_B_arr.append(T_T_B_)
    #     # robot.send_pose_to_robot(T_T_B_)

    #     # T_T_B_curr = robot.get_current_tool_pose()
    #     # T_G_S_curr = robot.T_B_S @ T_T_B_curr @ robot.T_G_T

    # robot.send_multiple_poses_to_robot(T_T_B_arr, at_once=True)


    # ik = IK('base_link', 'tool0', timeout=0.05, solve_type="Distance")
    # for i in range(T_G_0.shape[0]):
    #     T_G_S_ = T_G_0[i]
    #     T_T_B_ = robot.get_tool_pose_from_gripper_pose(T_G_S_)

    #     pose_ = matrix_to_pose(T_T_B_)
    #     t = pose_.position
    #     q = pose_.orientation
    #     # joints = np.array(robot.get_current_joint_values())
    #     joints = robot.get_current_joint_values()
        
    #     # goal_joints = IK.get_ik(qinit=joints, x=t.x, y=t.y, z=t.z, rx=q.x, ry=q.y, rz=q.z, rw=q.w)
    #     goal_joints = ik.get_ik(joints, t.x.item(), t.y.item(), t.z.item(), q.x.item(), q.y.item(), q.z.item(), q.w.item())

    #     robot.send_joint_values_to_robot(goal_joints)

    # joints_list = []
    # temp_joints = robot.get_current_joint_values()
    # for i in range(T_G_0.shape[0]):
    #     T_G_B_ = T_G_0[i]
    #     T_T_B_ = T_G_B_ @ np.linalg.inv(robot.T_G_T)
    #     # T_T_B_ = robot.get_tool_pose_from_gripper_pose(T_G_B_)
    #     pose_ = matrix_to_pose(T_T_B_)
    #     t = pose_.position
    #     q = pose_.orientation
    #     # joints = np.array(robot.get_current_joint_values())
    #     # joints = robot.get_current_joint_values()
        
    #     goal_joints = robot.ik.get_ik(temp_joints, t.x.item(), t.y.item(), t.z.item(), q.x.item(), q.y.item(), q.z.item(), q.w.item())
    #     joints_list.append(list(goal_joints))
    #     temp_joints = goal_joints

    # robot.send_multiple_joint_space_poses_to_robot(joints_list, wait=True)

    # RVL joints
    # joint_values_list[:, 0] %= 2*np.pi
    # joint_values_list[:, 0] = (joint_values_list[:, 0] + 2*np.pi) % 2*np.pi
    # joint_values_list = list(joint_values_list)

    # joint_values_list[joint_values_list[:,0]<-np.pi,0]+=(2.0*np.pi)
    # joint_values_list[joint_values_list[:,0]>np.pi,0]-=(2.0*np.pi)

    # joint_values_list[joint_values_list[:,5]<-np.pi,0]+=(2.0*np.pi)
    # joint_values_list[joint_values_list[:,5]>np.pi,0]-=(2.0*np.pi)
    """

    q[:, 0] += np.pi
    q[:, 5] += np.pi
    q[q>np.pi]-=(2.0*np.pi)     
    q[q<-np.pi]+=(2.0*np.pi)
    # q[0, :] = robot.joint_values_init

    # for i in range(q.shape[0]):
    #     joints = list(q[i])
    #     joints = [float(joint) for joint in joints]
    #     robot.send_joint_values_to_robot(joints)
        
    #     T_T_B = robot.get_current_tool_pose()
    #     T_G_B = T_T_B @ robot.T_G_T
    #     T_G_B_rvl = T_G_0[i]
    #     pass




    robot.send_multiple_joint_space_poses_to_robot(q, wait=True)
    # q_init[0] += np.pi
    # robot.send_multiple_joint_space_poses_to_robot([q_init], wait=True)
    

    # TEST
    # robot.send_multiple_joint_space_poses_to_robot([])


    # # Passing poses 
    # robot.send_multiple_poses_to_robot(T_T_B_arr, wait=True, cartesian=True)

    # rospy.sleep(1.5)
    # robot.send_named_pose('up')