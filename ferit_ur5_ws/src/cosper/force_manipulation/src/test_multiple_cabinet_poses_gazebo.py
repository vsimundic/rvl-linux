#!/usr/bin/python

import rospy
import os
import numpy as np
import json
import csv
import subprocess
from rospkg import RosPack
from tf2_ros import Buffer, TransformListener
from gazebo_msgs.msg import ContactsState
import sys
from core.util import read_config
from core.ur5_commander import UR5Commander
from core.transforms import rot_z
from gazebo_push_open.cabinet_model import Cabinet
from core.gazebo import get_link_pose
from force_utils import chebyshev_distance
from push_force_trajectories import generate_tool_line_poses, generate_trajectories
sys.path.append(os.path.join(os.path.dirname(__file__), '../../path_planning/src'))
from utils import contact_callback, kill_processes, reset_tf_buffer
import RVLPYDDManipulator as rvlpy

def generate_T_A_W_grid(
    x_range=(-0.18, 0.0), y_range=(0.45, 0.65),
    z_value=0.265, rot_range_deg=(-10, 10), step=0.05
):
    x_vals = np.arange(x_range[0], x_range[1] + step, step)
    y_vals = np.arange(y_range[0], y_range[1] + step, step)
    theta_deg = np.arange(rot_range_deg[0], rot_range_deg[1] + 5, 5)
    theta_rad = np.radians(theta_deg)

    X, Y, Theta = np.meshgrid(x_vals, y_vals, theta_rad, indexing='ij')
    num_transforms = X.size

    X_flat = X.ravel()
    Y_flat = Y.ravel()
    Theta_flat = Theta.ravel()

    cos_t = np.cos(Theta_flat)
    sin_t = np.sin(Theta_flat)

    R = np.zeros((num_transforms, 3, 3))
    R[:, 0, 0] = cos_t
    R[:, 0, 1] = -sin_t
    R[:, 1, 0] = sin_t
    R[:, 1, 1] = cos_t
    R[:, 2, 2] = 1.0

    T = np.zeros((num_transforms, 4, 4))
    T[:, :3, :3] = R
    T[:, 0, 3] = X_flat
    T[:, 1, 3] = Y_flat
    T[:, 2, 3] = z_value
    T[:, 3, 3] = 1.0

    return T

if __name__ == '__main__':
    rospy.init_node('single_cabinet_custom_trajectory_node')

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    cabinet_static_mesh_filename = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_static_mesh.ply'
    cabinet_panel_mesh_filename = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_panel_mesh.ply'
    cabinet_full_mesh_filename = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_full_mesh.ply'
    cabinet_urdf_save_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/cabinet_model/cabinet_model.urdf'
    load_trajs = False

    door_model_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/models/doorModel.json'
    with open(door_model_path, 'r') as f:
        data = json.load(f)

    R = np.array(data["R"])
    t = np.array(data["t"])
    s = np.array(data["s"])
    axis_pos = data["openingDirection"]
    T_A_C = np.eye(4)
    T_A_C[:3, :3] = R
    T_A_C[:3, 3] = t

    T_0_W = np.eye(4)
    T_0_W[2, 3] = 0.005
    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')
    rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

    door_thickness = 0.018
    static_depth = 0.4
    width = s[0]
    height = s[1]
    push_latch_mechanism_length = 0.046 + door_thickness * 0.5
    state_angle = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / width))

    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_cfg)
    rvl_manipulator.set_robot_pose(T_0_W)
    T_G_6 = rvl_manipulator.get_T_G_6()

    T_A_W_grid = generate_T_A_W_grid(step=0.05)

    kill_processes()
    reset_tf_buffer(tf_buffer)
    launch_process = subprocess.Popen("source /home/RVLuser/ferit_ur5_ws/devel/setup.bash && roslaunch ur5_robotiq_ft_3f_moveit_config demo_gazebo.launch", shell=True, executable="/bin/bash")
    rospy.sleep(4.)

    robot = UR5Commander()
    rospy.sleep(1.)

    R_TCP_D = np.array([[0, 0, -1], 
                        [0, 1, 0], 
                        [1, 0, 0]])
    T_TCP_G = np.eye(4)
    T_TCP_G[:3, 3] = np.array([0.155 / 2, 0, 0.098])
    T_G_DD_all = generate_tool_line_poses(0.5, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)
    i_TAW = 0
    for T_A_W in T_A_W_grid:
        cabinet_model = Cabinet(
            door_params=np.array([width, height, door_thickness, static_depth]),
            axis_pos=axis_pos,
            T_A_S=T_A_W,
            save_path=cabinet_urdf_save_path,
            has_handle=False)

        cabinet_model.save_mesh_without_doors(cabinet_static_mesh_filename)
        cabinet_model.save_door_panel_mesh(cabinet_panel_mesh_filename)
        cabinet_model.change_door_angle(state_angle)
        cabinet_model.update_mesh()
        cabinet_model.save_mesh(cabinet_full_mesh_filename)

        _, trajectories = generate_trajectories(T_G_DD_all, 37, state_angle, -90.0, cabinet_model, rvl_cfg, T_0_W, robot)

        if len(trajectories) == 0:
            print("No trajectories generated.")
            continue

        print(f"Generated {len(trajectories)} trajectory roots for T_A_W: {T_A_W[:3, 3]}")
        np.save('/home/RVLuser/ferit_ur5_ws/data/door_detection/trajectories/trajectories_{}.npy'.format(i_TAW), trajectories)
        i_TAW += 1
        # Load the trajectories
        trajs_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/trajectories/trajectories_{}.npy'.format(i_TAW)
        trajectories = np.load(trajs_path)
        # Execute each trajectory
        for i in range(trajectories.shape[0]):
            print(f"Executing trajectory {i}")

            path_np = np.array(trajectories[i])
            if not robot.validate_trajectory_points(path_np):
                continue
            
            # Extract contact point
            q_contact = path_np[0, :6]
            T_6_0_contact = robot.forward_kinematics(q_contact)
            T_6_0_contact_rvl = rvl_manipulator.fwd_kinematics_6(q_contact)
            
            if T_6_0_contact is None:
                print("Invalid trajectory, skipping...")
                continue
            
            T_G_W_contact = T_0_W @ T_6_0_contact @ T_G_6

            # Get approach path
            rvl_manipulator.set_environment_state(np.rad2deg(state_angle))
            T_G_0_via = rvl_manipulator.approach_path(T_G_W_contact)
            if T_G_0_via.shape[0] < 1:
                print("No points in approach path...")
                continue
            
            # Get approach path joint values
            approach_trajectory = [q_contact]
            for i_approach in range(T_G_0_via.shape[0]):
                T_G_0_via_ = T_G_0_via[i_approach]
                T_6_0_via_ = T_G_0_via_ @ np.linalg.inv(T_G_6)
                q_approach_, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0_via_)
                if n_sol < 1:
                    print("Invalid trajectory, skipping...")
                    break

                min_dist = np.inf
                best_sol = None
                for i_sol in range(n_sol):
                    q_sol = q_approach_[i_sol]
                    dist = chebyshev_distance(q_sol, approach_trajectory[0])
                    if dist > np.pi:
                        continue
                    if dist < min_dist and rvl_manipulator.free(q_sol) and robot.is_state_valid(q_sol):
                        best_sol = q_sol
                        min_dist = dist
                if best_sol is None:
                    print("Invalid trajectory, skipping...")
                    break
                approach_trajectory.insert(0, best_sol)
            approach_trajectory = np.array(approach_trajectory)
            
            if len(approach_trajectory[:-1]) < T_G_0_via.shape[0]:
                print("Invalid trajectory, skipping...")
                continue
            
            path_np[:, 0] -= np.pi
            path_np[:, 5] -= np.pi
            path_np[path_np > np.pi] -= 2 * np.pi
            path_np[path_np < -np.pi] += 2 * np.pi
            path_np = np.unwrap(path_np, axis=0)

            # Delete the model if it exists
            cabinet_model.delete_model_gazebo()

            # Create the approach path
            approach_path = [robot.get_current_joint_values(), path_np[0].tolist()]
            
            # Send the approach path to the robot
            robot.send_multiple_joint_space_poses_to_robot2(approach_path)
            
            # Spawn the cabinet model in Gazebo and set the door state
            cabinet_model.spawn_model_gazebo()
            cabinet_model.set_door_state_gazebo(state_angle)
            cabinet_model.change_door_angle(state_angle)

            # Send the opening trajectory to the robot
            robot.send_multiple_joint_space_poses_to_robot2(path_np.tolist())
            rospy.sleep(1.0)


    launch_process.terminate()
    launch_process.wait()
    kill_processes()
    print("Finished executing all T_A_W pose trajectories.")
