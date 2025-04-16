import numpy as np
import RVLPYDDManipulator as rvlpy
from DDMan.push import door_model, tool_model
from gazebo_push_open.cabinet_model import Cabinet
from core.transforms import rot_z
from force_utils import chebyshev_distance
import json
from tqdm import tqdm
from typing import Union
from core.ur5_commander import UR5Commander
from core.real_ur5_controller import UR5Controller
import copy
import open3d as o3d

def rotx_multiple(theta_arr):

    num_angles = theta_arr.shape[0]
    Rx = np.zeros((num_angles, 3, 3))
    c = np.cos(theta_arr)
    s = np.sin(theta_arr)
    Rx[:, 0, 0] = 1
    Rx[:, 1, 1] = c
    Rx[:, 1, 2] = -s
    Rx[:, 2, 1] = s
    Rx[:, 2, 2] = c
    return Rx

def generate_tool_line_poses(height, T_TCP_G=np.ndarray, R_TCP_D=np.ndarray):
    sample_dist = 0.02
    top_offset = 0.015
    side_offset = 0.015
    z_offset = 0.005

    num_samples = int(height / sample_dist)
    
    sample_pts_D = np.zeros((num_samples, 3))
    sample_pts_D[:, 1] = np.linspace(top_offset, height-top_offset, num_samples)
    sample_pts_D[:, 0] = -side_offset
    sample_pts_D[:, 2] = z_offset

    T_TCP_D = np.zeros((num_samples, 4, 4))
    T_TCP_D[:, 3, 3] = 1
    T_TCP_D[:, :3, :3] = R_TCP_D.copy()
    T_TCP_D[:, :3, 3] = sample_pts_D.copy()

    # Rotations around the X axis of G
    min_rotx_deg = -45.
    max_rotx_deg = 45.
    num_rot_samples = int((max_rotx_deg - min_rotx_deg)/5) + 1
    angles_deg = np.linspace(min_rotx_deg, max_rotx_deg, num_rot_samples)
    angles_rad = np.radians(angles_deg)
    Rx = rotx_multiple(angles_rad)

    T_Rx = np.zeros((num_rot_samples, 4, 4))
    T_Rx[:, 3, 3] = 1
    T_Rx[:, :3, :3] = Rx.copy()
    
    min_rotx_deg_first = -135.
    max_rotx_deg_first = 45.

    num_rot_samples_first = int((max_rotx_deg_first - min_rotx_deg_first)/5) + 1
    angles_first = np.linspace(min_rotx_deg_first, max_rotx_deg_first, num_rot_samples_first)
    angles_first_rad = np.radians(angles_first)
    Rx_first = rotx_multiple(angles_first_rad)
    T_Rx_first = np.zeros((num_rot_samples_first, 4, 4))
    T_Rx_first[:, 3, 3] = 1
    T_Rx_first[:, :3, :3] = Rx_first.copy()

    T_TCP_D_expanded_first = T_TCP_D[0] @ T_Rx_first
    T_G_D_arr_first = T_TCP_D_expanded_first @ np.linalg.inv(T_TCP_G)[np.newaxis, ...]

    T_TCP_D_expanded = T_TCP_D[:, np.newaxis, ...] @ T_Rx[np.newaxis, ...]
    T_TCP_D_expanded = T_TCP_D_expanded.reshape(-1, 4, 4) 
    
    T_G_D_arr = T_TCP_D_expanded @ np.linalg.inv(T_TCP_G)[np.newaxis, ...] # poses of G w.r.t D for each sample 
    
    T_G_D_arr_combined = np.concatenate((T_G_D_arr_first, T_G_D_arr), axis=0)

    return T_G_D_arr_combined

def get_all_leaves(node):
    leaves = []
    stack = [node]
    while stack:
        current = stack.pop()
        if current.is_leaf():
            leaves.append(current)
        else:
            stack.extend(current.children)
    return leaves

class Waypoint:
    def __init__(self, q, parent=None):
        self.q = np.array(q, dtype=np.float32)  
        self.parent = parent                    
        self.children = []                      
        self.dist_to_parent = 0.0 if parent is None else chebyshev_distance(q, parent.q)

    def add_child(self, child_q):
        child_node = Waypoint(child_q, parent=self)
        self.children.append(child_node)
        return child_node

    def is_leaf(self):
        return len(self.children) == 0

    def __repr__(self):
        return f"Waypoint(q={self.q.tolist()}, dist={self.dist_to_parent:.3f}, children={len(self.children)})"


def generate_trajectories(
    poses_G_DD: np.ndarray,
    num_states: int,
    init_state: float,
    goal_state: float,
    cabinet_model: Cabinet,
    rvl_ddmanipulator_config: str,
    T_0_W: np.ndarray,
    controller: Union[UR5Commander, UR5Controller]
):
    """
    Generate trajectories for each pose in poses_G_DD.

    Returns:
        - List of root Waypoints (trajectory trees)
        - Numpy array of joint trajectories with shape (N, num_states, 6)
    """
    # Generate states
    states = np.linspace(init_state, goal_state, num_states)
    states_rad = np.radians(states)

    # RVL manipulator object
    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_ddmanipulator_config)
    rvl_manipulator.set_robot_pose(T_0_W)
    rvl_manipulator.set_door_model_params(
        cabinet_model.d_door,
        cabinet_model.w_door,
        cabinet_model.h_door,
        cabinet_model.rx,
        cabinet_model.ry,
        cabinet_model.axis_pos,
        cabinet_model.static_side_width,
        cabinet_model.moving_to_static_part_distance)
    rvl_manipulator.set_door_pose(cabinet_model.T_A_S)

    all_trajectories = []
    traj_arrays = []

    T_W_0 = np.linalg.inv(T_0_W)
    T_G_6 = rvl_manipulator.get_T_G_6()
    T_6_G = np.linalg.inv(T_G_6)

    for T_G_D in tqdm(poses_G_DD, desc="Generating trajectories", unit="pose"):
        root_waypoints = []

        for i_state in tqdm(range(num_states), leave=False, desc="State steps", unit="step"):
            state_rad = states_rad[i_state]
            rvl_manipulator.set_environment_state(np.rad2deg(state_rad))

            T_A_A = np.eye(4)
            T_A_A[:3, :3] = rot_z(state_rad)

            T_6_0 = T_W_0 @ cabinet_model.T_A_S @ T_A_A @ cabinet_model.T_D_A @ T_G_D @ T_6_G
            T_G_W = T_0_W @ T_6_0 @ T_G_6

            joints, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0)
            # T_G_0 = T_6_0 @ T_G_6
            # joints, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols(T_6_0, False)
            # joints[:, 0] += np.pi
            # joints[joints > np.pi] -= (2.0 * np.pi)
            # joints[joints < -np.pi] += (2.0 * np.pi)
            if n_sol < 1:
                # print(f"No IK solution for state {i_state}, pose skipped.")
                break

            if i_state == 0:
                for i_sol in range(n_sol):
                    q = joints[i_sol, :]
                    # rvl_manipulator.visualize_current_state(q, T_6_0 @ T_G_6)
                    if rvl_manipulator.free(q) and rvl_manipulator.free_tool_only(T_G_W) and controller.is_state_valid(q.tolist()):
                        waypoint = Waypoint(q)
                        root_waypoints.append(waypoint)
            else:
                for root in root_waypoints:
                    leaves = [root] if root.is_leaf() else get_all_leaves(root)
                    for leaf in leaves:
                        best_q = None
                        min_dist = np.inf
                        for i_sol in range(n_sol):
                            q = joints[i_sol, :]
                            dist = chebyshev_distance(q, leaf.q)
                            if dist > 0.5*np.pi:
                                continue
                            if dist < min_dist and rvl_manipulator.free(q) and controller.is_state_valid(q.tolist()):
                                # rvl_manipulator.visualize_current_state(q, T_6_0 @ T_G_6)
                                best_q = q
                                min_dist = dist
                                # rvl_manipulator.visualize_current_state(q, T_6_0 @ T_G_6)
                        if best_q is not None:
                            leaf.add_child(best_q)

        for root in root_waypoints:
            trajectory = []
            node = root
            while node:
                trajectory.append(node.q)
                node = node.children[0] if node.children else None

            if len(trajectory) == num_states:
                all_trajectories.append(root)
                traj_arrays.append(np.stack(trajectory, axis=0))

    if traj_arrays:
        traj_array_np = np.stack(traj_arrays, axis=0)
        # traj_array_np[:, :, 0] -= np.pi
        # traj_array_np[traj_array_np > np.pi] -= 2 * np.pi
        # traj_array_np[traj_array_np < -np.pi] += 2 * np.pi
        # traj_array_np = np.unwrap(traj_array_np, axis=1)
        # traj_array_np = traj_array_np.astype(np.float32)
    else:
        traj_array_np = np.empty((0, num_states, 6), dtype=np.float32)

    return all_trajectories, traj_array_np


def generate_trajectories_and_approach(
    poses_G_DD: np.ndarray,
    num_states: int,
    init_state: float,
    goal_state: float,
    cabinet_model: Cabinet,
    rvl_ddmanipulator_config: str,
    T_0_W: np.ndarray,
    controller: Union[UR5Commander, UR5Controller]
):
    """
    Generate trajectories for each pose in poses_G_DD.

    Returns:
        - List of root Waypoints (trajectory trees)
        - Numpy array of joint trajectories with shape (N, num_states, 6)
    """
    states = np.linspace(init_state, goal_state, num_states)
    states_rad = np.radians(states)

    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_ddmanipulator_config)
    rvl_manipulator.set_robot_pose(T_0_W)
    rvl_manipulator.set_door_model_params(
        cabinet_model.d_door,
        cabinet_model.w_door,
        cabinet_model.h_door,
        cabinet_model.rx,
        cabinet_model.ry,
        cabinet_model.axis_pos,
        cabinet_model.static_side_width,
        cabinet_model.moving_to_static_part_distance)
    rvl_manipulator.set_door_pose(cabinet_model.T_A_S)

    all_trajectories = []
    traj_arrays = []

    T_W_0 = np.linalg.inv(T_0_W)
    T_G_6 = rvl_manipulator.get_T_G_6()
    T_6_G = np.linalg.inv(T_G_6)

    for T_G_D in tqdm(poses_G_DD, desc="Generating trajectories", unit="pose"):
        T_A_A = np.eye(4)
        T_A_A[:3, :3] = rot_z(states_rad[0])
        T_6_0_contact = T_W_0 @ cabinet_model.T_A_S @ T_A_A @ cabinet_model.T_D_A @ T_G_D @ T_6_G
        T_G_W_contact = T_0_W @ T_6_0_contact @ T_G_6
        
        rvl_manipulator.set_environment_state(np.rad2deg(states_rad[0]))
        T_G_0_via, ik_solutions, paths = rvl_manipulator.approach_path(T_G_W_contact.copy())
        if T_G_0_via.shape[0] < 1 or len(paths) < 1:
            continue

        root_waypoints = []
        for path in paths:
            ik0 = np.array(ik_solutions[0][path[0]])
            ik1 = np.array(ik_solutions[1][path[1]])
            for i_sol in range(len(ik_solutions[2])):
                ik2 = np.array(ik_solutions[2][i_sol])
                if chebyshev_distance(ik2, ik1) <= 0.5 * np.pi:
                    wp_root = Waypoint(ik0)
                    wp_mid = wp_root.add_child(ik1)
                    wp_mid.add_child(ik2)
                    root_waypoints.append(wp_root)

        if len(root_waypoints) == 0:
            continue

        for i_state in tqdm(range(num_states), leave=False, desc="State steps", unit="step"):
            if i_state == 0:
                continue  # Already initialized from approach path
            
            state_rad = states_rad[i_state]
            rvl_manipulator.set_environment_state(np.rad2deg(state_rad))

            T_A_A = np.eye(4)
            T_A_A[:3, :3] = rot_z(state_rad)

            T_6_0 = T_W_0 @ cabinet_model.T_A_S @ T_A_A @ cabinet_model.T_D_A @ T_G_D @ T_6_G
            T_G_W = T_0_W @ T_6_0 @ T_G_6

            joints, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0)
            if n_sol < 1:
                break

            for root in root_waypoints:
                leaves = [root] if root.is_leaf() else get_all_leaves(root)
                for leaf in leaves:
                    best_q = None
                    min_dist = np.inf
                    for i_sol in range(n_sol):
                        q = joints[i_sol, :]
                        dist = chebyshev_distance(q, leaf.q)
                        if dist > 0.5 * np.pi:
                            continue
                        if dist < min_dist and rvl_manipulator.free(q) and controller.is_state_valid(q.tolist()):
                            best_q = q
                            min_dist = dist
                        # else:
                        #     rvl_manipulator.visualize_current_state(q, T_6_0 @ T_G_6)
                    if best_q is not None:
                        leaf.add_child(best_q)

        for root in root_waypoints:
            trajectory = []
            node = root
            while node:
                trajectory.append(node.q)
                node = node.children[0] if node.children else None

            if len(trajectory) >= num_states+2:
                all_trajectories.append(root)
                traj_arrays.append(np.stack(trajectory, axis=0))

    if traj_arrays:
        traj_array_np = np.stack(traj_arrays, axis=0)
    else:
        traj_array_np = np.empty((0, num_states+2, 6), dtype=np.float32)

    return all_trajectories, traj_array_np


def generate_poses_demo():
    # --- Setup fixed transforms ---
    R_TCP_D = np.array([[0, 0, -1],
                        [0, 1, 0],
                        [1, 0, 0]])
    T_TCP_G = np.eye(4)
    T_TCP_G[:3, 3] = np.array([0.155 / 2, 0, 0.098])

    # Generate poses
    T_G_DD_all = generate_tool_line_poses(0.5, T_TCP_G, R_TCP_D)
    tool_poses = T_G_DD_all.reshape(-1, 4, 4)

    # --- Load Door Model ---
    vision_tolerance = 0.007
    door = door_model()
    door.dd_opening_direction = -1.0
    door.create(8.0, vision_tolerance)
    dd_mesh, dd_plate_mesh, dd_static_mesh = door.create_mesh()
    dd_plate_mesh.transform(np.linalg.inv(door.T_DD_W))

    # --- Load Gripper Model ---
    custom_gripper_spheres_path = '/home/RVLuser/rvl-linux/data/Robotiq3Finger/spheres.npy'
    custom_gripper_model_path = '/home/RVLuser/rvl-linux/data/Robotiq3Finger/mesh.ply'
    tool_contact_surface_params = np.array([[0.0, -0.026, 0.0], [0.0, -0.030, -0.020], [0.006, -0.026, 0.0]])
    tool_finger_distances = [-0.155 / 2., 0., -0.098]  # x, y, z
    sphere_to_TCS_distance = 0.004609
    gripper_params = {
        'is_default_gripper': False,
        'custom_gripper_spheres_path': custom_gripper_spheres_path,
        'custom_gripper_model_path': custom_gripper_model_path,
        'tool_contact_surface_params': tool_contact_surface_params,
        'tool_finger_distances': tool_finger_distances,
        'sphere_to_TCS_distance': sphere_to_TCS_distance
    }
    tool = tool_model(gripper_params)
    tool.create()
    tool_mesh = tool.create_mesh([0.5, 0.5, 0.5])


    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

    # Add static geometry
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
    vis.add_geometry(dd_plate_mesh)
    vis.add_geometry(origin_frame)

    # Add first tool instance
    tool_instance = copy.deepcopy(tool_mesh)
    tool_instance.transform(tool_poses[0])
    tool_instance.paint_uniform_color([0.5, 0.5, 0.5])
    tool_instance.compute_vertex_normals()
    vis.add_geometry(tool_instance)

    # Pose index tracker
    pose_idx = [0]

    # Update function
    def update_pose(vis):
        tool_instance.transform(np.linalg.inv(tool_poses[pose_idx[0]]))
        pose_idx[0] = (pose_idx[0] + 1) % len(tool_poses)
        tool_instance.transform(tool_poses[pose_idx[0]])
        vis.update_geometry(tool_instance)

    # Register key callbacks
    vis.register_key_callback(ord(" "), update_pose)

    print("Press SPACE step through poses.")
    vis.run()
    vis.destroy_window()

# Testing
def generate_trajectories_demo():
    # --- Setup fixed transforms ---
    R_TCP_D = np.array([[0, 0, -1],
                        [0, 1, 0],
                        [1, 0, 0]])
    T_TCP_G = np.eye(4)
    T_TCP_G[:3, 3] = np.array([0.155 / 2, 0, 0.098])

    # Generate poses
    T_G_DD_all = generate_tool_line_poses(0.5, T_TCP_G, R_TCP_D)
    T_G_DD_all = T_G_DD_all.reshape(-1, 4, 4)

    T_R_W = np.eye(4)
    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')

    # Load door model
    door_model_path = '/home/RVLuser/ferit_ur5_ws/data/door_detection/models/doorModel.json'
    with open(door_model_path, 'r') as f:
        data = json.load(f)
    
    R = np.array(data["R"])
    t = np.array(data["t"])
    s = np.array(data["s"])
    r = np.array(data["r"])
    axis_pos = data["openingDirection"]
    T_A_C = np.eye(4)
    T_A_C[:3, :3] = R
    T_A_C[:3, 3] = t

    # Load RVL manipulator config
    rvl_ddmanipulator_config = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
    T_0_W = np.eye(4)
    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_ddmanipulator_config)
    rvl_manipulator.set_robot_pose(T_0_W)

    joint_values = np.array(data["joint_values"])
    joint_values[0] += np.pi
    joint_values[joint_values > np.pi] -= (2.0 * np.pi)
    joint_values[joint_values < -np.pi] += (2.0 * np.pi)
    T_6_0 = rvl_manipulator.fwd_kinematics_6(joint_values)

    T_A_W = T_R_W @ T_6_0 @ T_C_6 @ T_A_C
    width = s[0]
    height = s[1]
	# Static cabinet params
    door_thickness=0.018
    static_depth=0.4
    push_latch_mechanism_length = 0.046 + door_thickness*0.5
    state_angle = axis_pos*np.rad2deg(np.arcsin(push_latch_mechanism_length/width))

    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                            axis_pos=axis_pos,
                            T_A_S=T_A_W,
                            has_handle=False)
    
    _, trajectories = generate_trajectories(T_G_DD_all, 37, state_angle, -90.0, cabinet_model, rvl_ddmanipulator_config, T_0_W)
    print(f"Generated {len(trajectories)} trajectories.")
    for traj in trajectories:
        print(traj)


if __name__ == "__main__":
    # generate_poses_demo()
    generate_trajectories_demo()