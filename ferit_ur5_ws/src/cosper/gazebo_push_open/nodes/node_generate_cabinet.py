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
from core.transforms import rot_z

if __name__ == '__main__':
    rospy.init_node('node_generate_cabinet')
    
    try:
        cfg_file = rospy.get_param('config_file')
    except rospy.exceptions.ROSException:
        raise Exception('Could not fetch param.')
    
    config = read_config(cfg_file)
    
    cabinet_dims = config['cabinet_dims'] # w_door, h_door, static_d, d_door 
    cabinet_pose = config['cabinet_pose']
    feasible_poses_args = config['feasible_poses']

    # Cabinet pose in world
    x = np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x'])
    y = np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y'])
    z = cabinet_dims[1]/2 + 0.018 + 0.01 # half of door height + depth of bottom panel + double static moving part distance
    spawn_angle_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])

    cabinet_model = Cabinet(door_params=np.array(cabinet_dims), 
                            axis_pos=cabinet_pose['axis_pos'],
                            position=np.array([x, y, z]),
                            spawn_angle_deg=spawn_angle_deg,
                            save_path=config['cabinet_urdf_save_path'])
    
    # Save cabinet mesh to a file
    cabinet_model.save_mesh(config['cabinet_mesh_save_path'])

    # Spawning model in Gazebo
    cabinet_model.delete_model_gazebo()
    cabinet_model.spawn_model_gazebo()

    # Open doors in Gazebo
    theta_deg = config['feasible_poses']['dd_state_deg']
    cabinet_model.open_door_gazebo(theta_deg)
    cabinet_model.change_door_angle(theta_deg)
    cabinet_model.update_mesh()

    # Get feasible poses
    feasible_poses = push.demo_push_poses_ros(**feasible_poses_args)
    
    # Pick one pose
    T_G_DD = feasible_poses[51]
        
    # Get robot gripper pose in world
    T_G0_0 = cabinet_model.get_feasible_pose_wrt_world(T_G_DD)
    
    # Pose further away for approach
    T_DD_G_prev = np.linalg.inv(T_G_DD.copy())
    T_DD_G_prev[2, 3] += 0.30
    T_G_DD_prev = np.linalg.inv(T_DD_G_prev)
    T_G0_0_prev = cabinet_model.get_feasible_pose_wrt_world(T_G_DD_prev)

    # # Open3D visualization
    # cabinet_model.visualize(T_G0_0)
    
    # Helping matrices
    T_B_0 = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TB0.npy')))
    T_G0_T = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TG0T.npy')))
    
    # T_G0_T = np.eye(4)
    # T_G0_T[:3, :3] = rot_z(np.radians(-90))
    # T_G0_T[2, 3] = 0.115
    # print(T_G0_T)
    # np.save(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TG0T.npy'), T_G0_T)
    
    # Robot handler
    robot = UR5Commander()

    # Go to home pose - up by default
    robot.send_named_pose('up')

    # Add cabinet mesh to rviz
    robot.add_mesh_to_scene(config['cabinet_mesh_save_path'], 'cabinet', np.linalg.inv(robot.T_B_0) @ cabinet_model.T_X_0)

    # Calculate poses
    T_T_B_fp_prev = robot.get_tool_pose(T_G0_0_prev) # approaching pose
    T_T_B_fp = robot.get_tool_pose(T_G0_0) # feasible pose
    T_G0_0_arr = cabinet_model.generate_opening_passing_poses(T_G_DD, max_angle_deg=70., num_poses=10) # passing poses
    T_T_B_arr = [robot.get_tool_pose(T_G_0_) for T_G_0_ in T_G0_0_arr]

    # Add door panel to planning scene to stop collision when approaching
    panel_rviz_name = 'door_panel'
    T_DD0_B = np.linalg.inv(T_B_0) @ cabinet_model.T_X_0 @ cabinet_model.T_A_X @ cabinet_model.T_DD0_A
    panel_size = (cabinet_model.d_door, cabinet_model.w_door, cabinet_model.h_door)
    robot.add_box_to_scene(object_name=panel_rviz_name, pose=T_DD0_B, frame_id='base_link', size=panel_size)

    # Approaching points - feasible pose
    robot.send_pose_to_robot(T_T_B_fp_prev)
    rospy.sleep(2)
    robot.send_pose_to_robot(T_T_B_fp, cartesian=True)
    rospy.sleep(2)

    # Remove panel from rviz because of the collision
    robot.remove_from_scene(name=panel_rviz_name)

    # Passing poses 
    robot.send_multiple_poses_to_robot(T_T_B_arr, wait=True, cartesian=True)

