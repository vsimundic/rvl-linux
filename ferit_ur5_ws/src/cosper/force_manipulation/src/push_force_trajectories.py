import numpy as np
import RVLPYDDManipulator as rvlpy
from DDMan.push import door_model, tool_model
from gazebo_push_open.cabinet_model import Cabinet
from gazebo_push_open.cabinet_model2 import Cabinet2
from core.transforms import rot_z, rot_y
from force_utils import chebyshev_distance
import json
from tqdm import tqdm
from typing import Union, List, Tuple
from core.ur5_commander import UR5Commander
from core.real_ur5_controller import UR5Controller
import copy
import open3d as o3d
import fcl
def load_fcl_mesh_from_mesh(mesh):
    """
    Load a mesh from a PLY file and create an FCL BVHModel.
    """

    # Extract vertices and triangles
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    triangles = np.asarray(mesh.triangles, dtype=np.int32)

    # Create FCL BVHModel
    fcl_mesh = fcl.BVHModel()
    fcl_mesh.beginModel(len(vertices), len(triangles))
    fcl_mesh.addSubModel(vertices, triangles)
    fcl_mesh.endModel()

    return fcl_mesh

def load_fcl_mesh_from_vertices_triangles(vertices, triangles):

    # Extract vertices and triangles
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int32)

    # Create FCL BVHModel
    fcl_mesh = fcl.BVHModel()
    fcl_mesh.beginModel(len(vertices), len(triangles))
    fcl_mesh.addSubModel(vertices, triangles)
    fcl_mesh.endModel()

    return fcl_mesh

def create_collision_object(mesh, pose):
    """
    Create a collision object for a mesh with a given pose.
    """
    # Convert pose to FCL-compatible format
    transform = fcl.Transform(pose[:3, :3], pose[:3, 3])
    # print("Transform rotation:\n", transform.getRotation())
    # print("Transform translation:\n", transform.getTranslation())
    return fcl.CollisionObject(mesh, transform)

def collision_detection_fcl(tool_mesh, tool_poses, cabinet_mesh, T_TCP_G: np.ndarray=None, static_mesh: o3d.geometry.TriangleMesh=None):

    # Tool mesh triangles, vertices
    tool_vertices = np.asarray(tool_mesh.vertices).copy()
    tool_triangles = np.asarray(tool_mesh.triangles).copy()

    # Cabinet preprocess - FCL
    T_plate_0 = np.eye(4)
    fcl_cabinet_mesh = load_fcl_mesh_from_mesh(cabinet_mesh)
    cabinet_col_obj = create_collision_object(fcl_cabinet_mesh, T_plate_0)

    if static_mesh:
        fcl_static_mesh = load_fcl_mesh_from_mesh(static_mesh)
        static_col_obj = create_collision_object(fcl_static_mesh, T_plate_0)
    else:
        static_col_obj = None

    # Initialize results list
    collision_results = []

    T_TCP_DD_list = []

    # Iterate through all tool poses
    # for pose in tqdm(tool_poses):
    num_poses = tool_poses.shape[0]
    # for idx, pose in tqdm(enumerate(tool_poses)):
    for idx in tqdm(range(num_poses)):
        T_G_DD = tool_poses[idx]

        fcl_tool_mesh = load_fcl_mesh_from_vertices_triangles(tool_vertices, tool_triangles)
        tool_col_obj = create_collision_object(fcl_tool_mesh, T_G_DD)

        dist_request = fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=True, gjk_solver_type=fcl.GJKSolverType.GST_INDEP)
        dist_result = fcl.DistanceResult()
        fcl.distance(cabinet_col_obj, tool_col_obj, dist_request, dist_result)
        # ret_dist = int(dist_result.min_distance < 0.0025)
        ret_dist = int(dist_result.min_distance < 0.0)
        # ret_dist = int(dist_result.min_distance < 0.0001)

        if static_col_obj:
            fcl.distance(static_col_obj, tool_col_obj, dist_request, dist_result)
            ret_static = int(dist_result.min_distance < 0.0001)
        else:
            ret_static = 0

        # Perform collision detection
        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        ret_col = fcl.collide(cabinet_col_obj, tool_col_obj, request, result)

        ## Debug
        # # if bool(ret) and not bool(collision[idx]):
        # if bool(collision[idx]) is False:
        #     if bool(ret) is True:
        #         tool_mesh_ = copy.deepcopy(tool_mesh)
        #         tool_mesh_.transform(T_G_DD)
        #         tool_mesh_.compute_vertex_normals()

        #         o3d.visualization.draw_geometries([tool_mesh_, cabinet_mesh])
        # if bool(ret) is True:
        #     if bool(collision[idx]) is False:
        #         tool_mesh_ = copy.deepcopy(tool_mesh)
        #         tool_mesh.paint_uniform_color([0.5, 0.5, 0])
        #         tool_mesh_.transform(T_G_DD)
        #         tool_mesh_.compute_vertex_normals()

        #         o3d.visualization.draw_geometries([tool_mesh_, cabinet_mesh])

        # Store the result for the current pose
        # if not ret_dist and ret_col:
        #     debug = 0
        #     dist_request = fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=True, gjk_solver_type=fcl.GJKSolverType.GST_INDEP)
        #     dist_result = fcl.DistanceResult()
        #     fcl.distance(cabinet_col_obj, tool_col_obj, dist_request, dist_result)

        # if idx == 5000:
        # else:
        #     dist_request = fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=True, gjk_solver_type=fcl.GJKSolverType.GST_INDEP)
        #     dist_result = fcl.DistanceResult()
        #     fcl.distance(cabinet_col_obj, tool_col_obj, dist_request, dist_result)
        #     pass

        if not ret_dist and not ret_static:
            collision_results.append(0)
        else:
            collision_results.append(1)
            # tool_mesh_ = copy.deepcopy(tool_mesh)
            # tool_mesh_.transform(T_G_DD)
            # tool_mesh_.compute_vertex_normals()
            # origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
            # o3d.visualization.draw_geometries([tool_mesh_, cabinet_mesh, origin_rf] + [static_mesh] if static_mesh else [])
    return collision_results

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

def roty_multiple(theta_arr):
    num_angles = theta_arr.shape[0]
    Ry = np.zeros((num_angles, 3, 3))
    c = np.cos(theta_arr)
    s = np.sin(theta_arr)
    Ry[:, 0, 0] = c
    Ry[:, 0, 2] = s
    Ry[:, 1, 1] = 1
    Ry[:, 2, 0] = -s
    Ry[:, 2, 2] = c
    return Ry

def rotz_multiple(theta_arr):
    num_angles = theta_arr.shape[0]
    Rz = np.zeros((num_angles, 3, 3))
    c = np.cos(theta_arr)
    s = np.sin(theta_arr)
    Rz[:, 0, 0] = c
    Rz[:, 0, 1] = -s
    Rz[:, 1, 0] = s
    Rz[:, 1, 1] = c
    Rz[:, 2, 2] = 1
    return Rz

def generate_tool_line_poses(height, T_TCP_G=np.ndarray, R_TCP_D=np.ndarray):
    sample_dist = 0.02
    top_offset = 0.0115
    side_offset = 0.0115
    z_offset = 0.007

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
    min_rotx_deg = 45.
    max_rotx_deg = -45.
    num_rot_samples = int(abs(max_rotx_deg - min_rotx_deg)/5) + 1
    angles_deg = np.linspace(min_rotx_deg, max_rotx_deg, num_rot_samples)
    angles_rad = np.radians(angles_deg)
    Rx = rotx_multiple(angles_rad)

    # Rotation samples around Y
    # min_roty_deg, max_roty_deg = 5, 45.
    min_roty_deg, max_roty_deg = 35., -35. # 1 finger
    num_rot_samples_y = int(abs(max_roty_deg - min_roty_deg)/2) + 1
    angles_y_rad = np.radians(np.linspace(min_roty_deg, max_roty_deg, num_rot_samples_y))
    Ry = roty_multiple(angles_y_rad)

    Rx_exp = Rx[:, np.newaxis, :, :]
    Ry_exp = Ry[np.newaxis, :, :, :]
    R_combined = np.matmul(Rx_exp, Ry_exp)

    N_combined = R_combined.shape[0] * R_combined.shape[1]

    T_rotations = np.tile(np.eye(4), (N_combined, 1, 1))
    T_rotations[:, :3, :3] = R_combined.reshape(N_combined, 3, 3)

    # T_Rx = np.zeros((num_rot_samples, 4, 4))
    # T_Rx[:, 3, 3] = 1
    # T_Rx[:, :3, :3] = Rx.copy()

    min_rotx_deg_first = 135.
    max_rotx_deg_first = min_rotx_deg - 5.

    num_rot_samples_first = int(abs(max_rotx_deg_first - min_rotx_deg_first)/5) + 1
    angles_first = np.linspace(min_rotx_deg_first, max_rotx_deg_first, num_rot_samples_first)
    angles_first_rad = np.radians(angles_first)
    Rx_first = rotx_multiple(angles_first_rad)

    Rx_first_exp = Rx_first[:, np.newaxis, :, :]  # (N_first_x, 1, 3, 3)
    Ry_first_exp = Ry[np.newaxis, :, :, :]        # (1, N_y, 3, 3)
    R_first_combined = np.matmul(Rx_first_exp, Ry_first_exp)  # (N_first_x, N_y, 3, 3)
    N_first_combined = R_first_combined.shape[0] * R_first_combined.shape[1]

    T_rotations_first = np.tile(np.eye(4), (N_first_combined, 1, 1))
    T_rotations_first[:, :3, :3] = R_first_combined.reshape(N_first_combined, 3, 3)

    # T_Rx_first = np.zeros((num_rot_samples_first, 4, 4))
    # T_Rx_first[:, 3, 3] = 1
    # T_Rx_first[:, :3, :3] = Rx_first.copy()

    # T_TCP_D_expanded_first = T_TCP_D[0] @ T_Rx_first
    T_TCP_D_expanded_first = T_TCP_D[0] @ T_rotations_first
    T_G_D_arr_first = T_TCP_D_expanded_first @ np.linalg.inv(T_TCP_G)[np.newaxis, ...]

    # T_TCP_D_expanded = T_TCP_D[:, np.newaxis, ...] @ T_Rx[np.newaxis, ...]
    T_TCP_D_expanded = T_TCP_D[:, np.newaxis, ...] @ T_rotations[np.newaxis, ...]
    T_TCP_D_expanded = T_TCP_D_expanded.reshape(-1, 4, 4)

    T_G_D_arr = T_TCP_D_expanded @ np.linalg.inv(T_TCP_G)[np.newaxis, ...] # poses of G w.r.t D for each sample

    T_G_D_arr_combined = np.concatenate((T_G_D_arr_first, T_G_D_arr), axis=0)
    return T_G_D_arr_combined
    # return T_G_D_arr

def visualize_poses(tool_mesh, cabinet_mesh, T_G_D_arr, T_TCP_G):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name='Pose Visualizer', width=800, height=600)

    origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    TCP_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)
    cabinet_mesh_ = copy.deepcopy(cabinet_mesh)
    tool_mesh_ = copy.deepcopy(tool_mesh)
    tool_mesh_.compute_vertex_normals()

    vis.add_geometry(cabinet_mesh_)
    vis.add_geometry(origin_rf)
    vis.add_geometry(tool_mesh_)
    vis.add_geometry(TCP_rf)

    # State index
    state = {"i": 0}

    def next_pose(vis):
        i = state["i"]
        if i < len(T_G_D_arr):
            T_G_D = T_G_D_arr[i]
            T_TCP_D = T_G_D @ T_TCP_G

            tool_mesh_.transform(T_G_D)
            vis.update_geometry(tool_mesh_)

            TCP_rf.transform(T_TCP_D)
            vis.update_geometry(TCP_rf)

            vis.poll_events()
            vis.update_renderer()
            tool_mesh_.transform(np.linalg.inv(T_G_D))  # Undo for next transform
            TCP_rf.transform(np.linalg.inv(T_TCP_D))  # Undo for next transform
            state["i"] += 1
        return False  # False means keep window open

    # Spacebar = 32, right arrow = 262
    vis.register_key_callback(32, next_pose)     # Spacebar
    vis.register_key_callback(262, next_pose)    # Right arrow

    print("Press SPACE or RIGHT ARROW to step through poses. Close window to exit.")
    vis.run()
    vis.destroy_window()


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
    def __init__(self, q, T_6_0, parent=None):
        self.q = np.array(q, dtype=np.float32)
        self.T_6_0 = T_6_0
        self.parent = parent # type: Waypoint
        self.children = [] # type: List[Waypoint]
        self.dist_to_parent = 0.0 if parent is None else chebyshev_distance(q, parent.q)
        self.depth = 0 if parent is None else parent.depth + 1

    def add_child(self, child_q, T_6_0) -> 'Waypoint':
        child_node = Waypoint(child_q, T_6_0, parent=self)
        self.children.append(child_node)
        return child_node

    def is_leaf(self) -> bool:
        return len(self.children) == 0

    def __repr__(self):
        return f"Waypoint(q={self.q.tolist()}, dist={self.dist_to_parent:.3f}, children={len(self.children)})"

    def get_full_trajectory_joint_space(self) -> np.ndarray:
        trajectory = []
        node = self
        # Get to the root node and then make a trajectory
        node_root = None
        while node:
            node_root = node
            node = node.parent
        # Now traverse down from the root to the end
        node = node_root
        while node:
            trajectory.append(node.q)
            node = node.children[0] if node.children else None

        return np.array(trajectory)

    def get_full_trajectory_task_space(self) -> np.ndarray:
        trajectory = []
        node = self
        # Get to the root node and then make a trajectory
        node_root = None
        while node:
            node_root = node
            node = node.parent
        # Now traverse down from the root to the end
        node = node_root
        while node:
            trajectory.append(node.T_6_0)
            node = node.children[0] if node.children else None

        return np.array(trajectory)

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

            T_Arot_A = np.eye(4)
            T_Arot_A[:3, :3] = rot_z(state_rad)

            T_6_0 = T_W_0 @ cabinet_model.T_A_S @ T_Arot_A @ cabinet_model.T_D_A_init @ T_G_D @ T_6_G
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
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(states_rad[0])
        T_6_0_contact = T_W_0 @ cabinet_model.T_A_S @ T_Arot_A @ cabinet_model.T_D_A_init @ T_G_D @ T_6_G
        T_G_W_contact = T_0_W @ T_6_0_contact @ T_G_6

        rvl_manipulator.set_environment_state(np.rad2deg(states_rad[0]))
        q = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0_contact)[0][0]
        # rvl_manipulator.visualize_current_state(q, T_6_0_contact @ T_G_6)

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

            T_Arot_A = np.eye(4)
            T_Arot_A[:3, :3] = rot_z(state_rad)

            T_6_0 = T_W_0 @ cabinet_model.T_A_S @ T_Arot_A @ cabinet_model.T_D_A_init @ T_G_D @ T_6_G
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


def generate_trajectories_and_approach2(
    poses_G_DD: np.ndarray,
    num_states: int,
    init_state: float,
    goal_state: float,
    cabinet_model: Cabinet2,
    # rvl_ddmanipulator_config: str,
    rvl_manipulator: rvlpy.PYDDManipulator,
    T_0_W: np.ndarray,
    controller: Union[UR5Commander, UR5Controller],
    verbose: bool = False,
    visualize: bool = False) -> Tuple[List[Waypoint], np.ndarray]:
    """
    Generate trajectories for each pose in poses_G_DD.

    Returns:
        - List of root Waypoints (trajectory trees)
        - Numpy array of joint trajectories with shape (N, num_states, 6)
    """
    states = np.linspace(init_state, goal_state, num_states)
    states_rad = np.radians(states)
    all_trajectories = []
    traj_arrays = []

    T_W_0 = np.linalg.inv(T_0_W)
    T_G_6 = rvl_manipulator.get_T_G_6()
    T_6_G = np.linalg.inv(T_G_6)

    approach_error_labels = {
        0: "Approach success",
        1: "No IK solution for contact pose",
        2: "Invalid sphere pose",
        3: "Collision between contact and via",
        4: "Via point 1 IK failure",
        5: "Via point 0 IK failure",
        6: "No valid IK paths",
        7: "Cylinder collision",
        8: "Contact sphere collision"
    }

    # debug
    if visualize:
        contact_poses_idx = [0, 10, 20, 30]
    else:
        contact_poses_idx = []

    # Diagnostic counters
    num_total_poses = 0
    num_successful_trajectories = 0
    num_failed_ik = 0
    num_failed_root_init = 0
    num_failed_leaf_growth = 0
    num_failed_approach = [0] * len(approach_error_labels)
    not_enough_traj_pts = 0

    if visualize:
        T_G_D = poses_G_DD[0]
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(states_rad[0])
        T_6_0_contact = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

        # q_rvl_debug = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # just to make the type checker happy
        # rvl_manipulator.visualize_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)
    visualized = False

    for i_pose in tqdm(
        range(poses_G_DD.shape[0]),
        desc="Generating trajectories",
        unit="pose",
        disable=not verbose):

        T_G_D = poses_G_DD[i_pose]
        num_total_poses += 1
        root_waypoints = [] # type: List[Waypoint]
        pose_failed = False
        pose_logged = False

        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(states_rad[0])
        T_6_0_contact = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G
        T_G_W_contact = T_0_W @ T_6_0_contact @ T_G_6

        rvl_manipulator.set_environment_state(np.rad2deg(states_rad[0]))

        if visualize:
            if i_pose in contact_poses_idx:
                q_rvl_debug = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # just to make the type checker happy
                rvl_manipulator.visualize_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)

        # Get approach path
        T_G_0_via, ik_solutions, paths, error_code = rvl_manipulator.approach_path(T_G_W_contact.copy())
        if T_G_0_via.shape[0] < 2 or len(paths) < 1:
            num_failed_approach[error_code] += 1
            pose_logged = True
            continue
        T_6_0_via = T_G_0_via @ T_6_G[np.newaxis, ...]

        # Adjust IK solutions to ROS and limit ranges
        for ik_sols in ik_solutions:
            ik_sols[:, 0] -= np.pi
            ik_sols[:, 5] -= np.pi
            ik_sols[ik_sols > np.pi] -= (2.0 * np.pi)
            ik_sols[ik_sols < -np.pi] += (2.0 * np.pi)

        # Check for paths in the approach path and add them to the root waypoints
        for path in paths:
            ik0 = np.array(ik_solutions[0][path[0]])
            ik1 = np.array(ik_solutions[1][path[1]])

            for i_sol in range(len(ik_solutions[2])):
                ik2 = np.array(ik_solutions[2][i_sol]) # contact point
                if chebyshev_distance(ik2, ik1) <= 0.5 * np.pi and \
                    controller.is_state_valid(ik2.tolist()) and \
                    controller.is_state_valid(ik1.tolist()) and \
                    controller.is_state_valid(ik0.tolist()):
                    wp_root = Waypoint(ik0, T_6_0_via[0])
                    wp_mid = wp_root.add_child(ik1, T_6_0_via[1])
                    wp_mid.add_child(ik2, T_6_0_contact)
                    root_waypoints.append(wp_root)

        # If no root waypoints were found, skip this pose
        if len(root_waypoints) == 0:
            if not pose_logged:
                num_failed_root_init += 1
                pose_logged = True
            continue

        # Iterate through the states and try to grow the tree
        for i_state in range(1, num_states):
            state_rad = states_rad[i_state]
            rvl_manipulator.set_environment_state(np.rad2deg(state_rad))

            T_Arot_A = np.eye(4)
            T_Arot_A[:3, :3] = rot_z(state_rad)
            T_6_0 = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

            joints: np.ndarray # just to make the type checker happy
            joints, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0)
            if n_sol < 1:
                if not pose_logged:
                    num_failed_ik += 1
                    pose_logged = True
                pose_failed = True
                break
            joints = joints[~np.isnan(joints).any(axis=1)]  # Remove NaN rows
            mask = ~((joints[:, 1] > 0.) & (joints[:, 1] < np.pi))
            joints = joints[mask]
            n_sol = joints.shape[0]

            joints_rvl = joints.copy()
            for i_sol in range(n_sol):
                q = joints[i_sol, :]
                q[0] -= np.pi
                q[5] -= np.pi
                q[q > np.pi] -= (2.0 * np.pi)
                q[q < -np.pi] += (2.0 * np.pi)

            # For each root waypoint, try to grow the tree
            # If the root is a leaf, we can add the new state directly
            for root in root_waypoints:
                leaves = [root] if root.is_leaf() else get_all_leaves(root)
                for leaf in leaves:
                    best_q = None
                    min_dist = np.inf
                    for i_sol in range(n_sol):
                        q = joints[i_sol, :]
                        q_rvl = joints_rvl[i_sol, :]
                        dist = chebyshev_distance(q, leaf.q)
                        if dist > 0.5 * np.pi:
                            continue
                        if visualize and not visualized:
                            rvl_manipulator.visualize_current_state(q_rvl, T_6_0 @ T_G_6)
                            visualized = True
                        if dist < min_dist and rvl_manipulator.free(q_rvl) and controller.is_state_valid(q.tolist()):
                            best_q = q
                            min_dist = dist
                    if best_q is not None:
                        leaf.add_child(best_q, T_6_0)

        if not pose_failed:
            any_valid = False
            for i_root, root in enumerate(root_waypoints):
                trajectory = []
                node = root
                while node:
                    trajectory.append(node.q)
                    node = node.children[0] if node.children else None
                if len(trajectory) >= num_states + 2:
                    all_trajectories.append(root)
                    traj_arrays.append(np.stack(trajectory, axis=0))
                    num_successful_trajectories += 1
                    any_valid = True
            if not any_valid and not pose_logged:
                not_enough_traj_pts += 1
                pose_logged = True

    if verbose:
        print("Trajectory Generation Summary:")
        print(f"  Total Poses: {num_total_poses}")
        print(f"  Successful Trajectories: {num_successful_trajectories}")
        print(f"  Failures:")
        print(f"    - No IK solution: {num_failed_ik}")
        print(f"    - No valid root state: {num_failed_root_init}")
        print(f"    - No leaf could grow: {num_failed_leaf_growth}")
        print(f"    - Not enough traj points: {not_enough_traj_pts}")
        print(f"    - Approach path failures by error code:")
        for i, count in enumerate(num_failed_approach):
            if count > 0:
                label = approach_error_labels.get(i, "Unknown error")
                print(f"      Code {i} ({label}): {count} times")

    if traj_arrays:
        traj_array_np = np.stack(traj_arrays, axis=0)
    else:
        traj_array_np = np.empty((0, num_states + 2, 6), dtype=np.float32)

    return all_trajectories, traj_array_np


def generate_trajectories_and_approach3(
    poses_G_DD: np.ndarray,
    num_states: int,
    init_state: float,
    goal_state: float,
    cabinet_model: Cabinet2,
    # rvl_ddmanipulator_config: str,
    rvl_manipulator: rvlpy.PYDDManipulator,
    T_0_W: np.ndarray,
    controller: Union[UR5Commander, UR5Controller],
    verbose: bool = False,
    visualize: bool = False,
    T_D_S: np.ndarray = None,
    T_TCP_6: np.ndarray = None
    ) -> Tuple[List[Waypoint], np.ndarray]:

    """
    Generate trajectories for each pose in poses_G_DD.

    Returns:
        - List of root Waypoints (trajectory trees)
        - Numpy array of joint trajectories with shape (N, num_states, 6)
    """
    
    states = np.linspace(init_state, goal_state, num_states)
    states_rad = np.radians(states)
    all_trajectories = []
    traj_arrays = []

    T_W_0 = np.linalg.inv(T_0_W)
    # T_G_6 = rvl_manipulator.get_T_G_6()
    # T_6_G = np.linalg.inv(T_G_6)
    T_G_6 = controller.T_G_6
    T_6_G = np.linalg.inv(T_G_6)

    approach_error_labels = {
        0: "Approach success",
        1: "No IK solution for contact pose",
        2: "Invalid sphere pose",
        3: "Collision between contact and via",
        4: "Via point 1 IK failure",
        5: "Via point 0 IK failure",
        6: "No valid IK paths",
        7: "Cylinder collision",
        8: "Contact sphere collision"
    }

    # debug
    contact_poses_idx = [0]
    if not visualize:
        contact_poses_idx = []

    # Diagnostic counters
    num_total_poses = 0
    num_successful_trajectories = 0
    num_failed_ik = 0
    num_failed_root_init = 0
    num_failed_leaf_growth = 0
    num_failed_approach = [0] * len(approach_error_labels)
    not_enough_traj_pts = 0

    # # if visualize:
    T_G_D = poses_G_DD[1000]
    T_Arot_A = np.eye(4)
    T_Arot_A[:3, :3] = rot_z(states_rad[0])
    T_6_0_contact = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

    q_rvl_debug = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # just to make the type checker happy
    # # rvl_manipulator.visualize_vn_model()
    # # rvl_manipulator.visualize_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)
    # rvl_manipulator.visualize_vn_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)

    visualized = False

    for i_pose in tqdm(
        range(poses_G_DD.shape[0]),
        desc="Generating trajectories",
        unit="pose",
        disable=not verbose):

        T_G_D = poses_G_DD[i_pose]
        num_total_poses += 1
        root_waypoints = [] # type: List[Waypoint]
        pose_failed = False
        pose_logged = False

        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(states_rad[0])
        T_6_0_contact = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G
        T_G_W_contact = T_0_W @ T_6_0_contact @ T_G_6

        # rvl_manipulator.visualize_vn_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)
        # rvl_manipulator.set_environment_state(np.rad2deg(states_rad[0]))

        if visualize:
            if i_pose in contact_poses_idx:
                q_rvl_debug = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]) # just to make the type checker happy
                # rvl_manipulator.visualize_vn_model()
                # rvl_manipulator.visualize_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)
                # rvl_manipulator.visualize_vn_model()
                # rvl_manipulator.visualize_vn_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)

                # # o3d visualization
                # cabinet_model_ = copy.deepcopy(cabinet_model)
                # cabinet_model_.change_door_angle(np.rad2deg(states_rad[0]))
                # cabinet_model_.update_mesh()
                # filename = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/cabinet_model.ply'
                # cabinet_model_.save_mesh(filename)

                # cabinet_mesh = o3d.io.read_triangle_mesh(filename)
                # cabinet_mesh.compute_vertex_normals()
                # cabinet_mesh.transform(cabinet_model_.T_A_S @ T_Arot_A)

                # A_S_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                # A_S_rf.transform(cabinet_model_.T_A_S @ T_Arot_A)

                # DD_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                # DD_rf.transform(cabinet_model_.T_A_S @ T_Arot_A @ cabinet_model_.T_D_A_init)

                # gripper_mesh = o3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply')
                # gripper_mesh.transform(T_6_0_contact @ T_G_6)
                # gripper_mesh.compute_vertex_normals()
                # o3d.visualization.draw_geometries([cabinet_mesh, gripper_mesh, DD_rf, A_S_rf],
                #                                   window_name='Pose Visualizer',
                #                                   width=800, height=600)

        # Get approach path
        T_G_0_via, ik_solutions, paths, error_code = rvl_manipulator.approach_path(T_G_W_contact.copy())
        if T_G_0_via.shape[0] < 2 or len(paths) < 1:
            if error_code in range(len(approach_error_labels)):
                num_failed_approach[error_code] += 1
            pose_logged = True
            continue
        T_6_0_via = T_G_0_via @ T_6_G[np.newaxis, ...]

        # Adjust IK solutions to ROS and limit ranges
        for ik_sols in ik_solutions:
            ik_sols[:, 0] -= np.pi
            ik_sols[:, 5] -= np.pi
            ik_sols[ik_sols > np.pi] -= (2.0 * np.pi)
            ik_sols[ik_sols < -np.pi] += (2.0 * np.pi)

        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(states_rad[0])
        T_Arot_0 = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A

        T_6_0 = T_Arot_0 @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

        cabinet_model.change_door_angle(np.rad2deg(states_rad[0]))
        cabinet_model.create_mesh()
        cabinet_model.save_mesh()
        controller.add_mesh_to_scene(cabinet_model.mesh_save_path, 'cabinet_model', T_Arot_0, verbose=False)


        # Check for paths in the approach path and add them to the root waypoints
        for path in paths:
            ik0 = np.array(ik_solutions[0][path[0]])
            ik1 = np.array(ik_solutions[1][path[1]])

            for i_sol in range(len(ik_solutions[2])):
                ik2 = np.array(ik_solutions[2][i_sol]) # contact point
                # T_6_0_contact_ = controller.get_fwd_kinematics_moveit(ik2)
                # T_TCP_D = np.linalg.inv(T_D_S) @ T_6_0_contact_ @ T_TCP_6

                if chebyshev_distance(ik2, ik1) <= 0.5 * np.pi and \
                    controller.is_state_valid(ik2.tolist()) and \
                    controller.is_state_valid(ik1.tolist()) and \
                    controller.is_state_valid(ik0.tolist()):
                    wp_root = Waypoint(ik0, T_6_0_via[0])
                    wp_mid = wp_root.add_child(ik1, T_6_0_via[1])
                    wp_mid.add_child(ik2, T_6_0_contact)
                    root_waypoints.append(wp_root)

        # If no root waypoints were found, skip this pose
        if len(root_waypoints) == 0:
            if not pose_logged:
                num_failed_root_init += 1
                pose_logged = True
            continue

        # Iterate through the states and try to grow the tree
        for i_state in range(1, num_states):
            state_rad = states_rad[i_state]
            # rvl_manipulator.set_environment_state(np.rad2deg(state_rad))

            T_Arot_A = np.eye(4)
            T_Arot_A[:3, :3] = rot_z(state_rad)
            T_Arot_0 = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A

            T_6_0 = T_Arot_0 @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

            cabinet_model.change_door_angle(np.rad2deg(state_rad))
            cabinet_model.create_mesh()
            cabinet_model.save_mesh()
            controller.add_mesh_to_scene(cabinet_model.mesh_save_path, 'cabinet_model', T_Arot_0, verbose=False)

            joints: np.ndarray # just to make the type checker happy
            joints, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0)
            if n_sol < 1:
                if not pose_logged:
                    num_failed_ik += 1
                    pose_logged = True
                pose_failed = True
                break
            joints = joints[~np.isnan(joints).any(axis=1)]  # Remove NaN rows
            mask = ~((joints[:, 1] > 0.) & (joints[:, 1] < np.pi))
            joints = joints[mask]
            n_sol = joints.shape[0]

            joints_rvl = joints.copy()
            for i_sol in range(n_sol):
                q = joints[i_sol, :]
                q[0] -= np.pi
                # q[5] -= np.pi
                q[q > np.pi] -= (2.0 * np.pi)
                q[q < -np.pi] += (2.0 * np.pi)

            # For each root waypoint, try to grow the tree
            # If the root is a leaf, we can add the new state directly
            for root in root_waypoints:
                leaves = [root] if root.is_leaf() else get_all_leaves(root)
                for leaf in leaves:
                    best_q = None
                    min_dist = np.inf
                    for i_sol in range(n_sol):
                        q = joints[i_sol, :]
                        q_rvl = joints_rvl[i_sol, :]
                        dist = chebyshev_distance(q, leaf.q)
                        if dist > 0.5 * np.pi:
                            continue
                        if visualize and not visualized:
                            # rvl_manipulator.visualize_current_state(q_rvl, T_6_0 @ T_G_6)
                            rvl_manipulator.visualize_vn_current_state(q_rvl, T_6_0 @ T_G_6)
                            visualized = True
                        # if dist < min_dist and rvl_manipulator.free(q_rvl) and controller.is_state_valid(q.tolist()):
                        if dist < min_dist and controller.is_state_valid(q.tolist()):
                            best_q = q
                            min_dist = dist
                    if best_q is not None:
                        leaf.add_child(best_q, T_6_0)

        if not pose_failed:
            any_valid = False
            for i_root, root in enumerate(root_waypoints):
                trajectory = []
                node = root
                while node:
                    trajectory.append(node.q)
                    node = node.children[0] if node.children else None
                if len(trajectory) >= num_states + 2:
                    all_trajectories.append(root)
                    trajectory = np.unwrap(trajectory, axis=0)
                    traj_arrays.append(np.stack(trajectory, axis=0))
                    num_successful_trajectories += 1
                    any_valid = True
            if not any_valid and not pose_logged:
                not_enough_traj_pts += 1
                pose_logged = True



    if verbose:
        print("Trajectory Generation Summary:")
        print(f"  Total Poses: {num_total_poses}")
        print(f"  Successful Trajectories: {num_successful_trajectories}")
        print(f"  Failures:")
        print(f"    - No IK solution: {num_failed_ik}")
        print(f"    - No valid root state: {num_failed_root_init}")
        print(f"    - No leaf could grow: {num_failed_leaf_growth}")
        print(f"    - Not enough traj points: {not_enough_traj_pts}")
        print(f"    - Approach path failures by error code:")
        for i, count in enumerate(num_failed_approach):
            if count > 0:
                label = approach_error_labels.get(i, "Unknown error")
                print(f"      Code {i} ({label}): {count} times")

    if traj_arrays:
        traj_array_np = np.stack(traj_arrays, axis=0)

        # traj_ = all_trajectories[0] if len(all_trajectories) > 0 else None
        # T_6_0_traj = traj_.get_full_trajectory_task_space()[:3, ...]
        # for i in range(0, 3):
        #     T_6_0_ = T_6_0_traj[i, ...]
        #     T_G_0_ = T_6_0_ @ T_G_6
        #     rvl_manipulator.visualize_vn_current_state(q_rvl_debug, T_G_0_)
    else:
        traj_array_np = np.empty((0, num_states + 2, 6), dtype=np.float32)

    return all_trajectories, traj_array_np


def generate_one_trajectory_and_approach(
    poses_G_DD: np.ndarray,
    num_states: int,
    init_state: float,
    goal_state: float,
    cabinet_model: Cabinet2,
    # rvl_ddmanipulator_config: str,
    rvl_manipulator: rvlpy.PYDDManipulator,
    T_0_W: np.ndarray,
    controller: Union[UR5Commander, UR5Controller],
    verbose: bool = True
):
    """
    Generate a single trajectory for the first pose in poses_G_DD.

    Returns:
        - List of root Waypoints (trajectory trees)
        - Numpy array of joint trajectories with shape (N, num_states, 6)
    """
    states = np.linspace(init_state, goal_state, num_states)
    states_rad = np.radians(states)
    all_trajectories = []
    traj_arrays = []

    T_W_0 = np.linalg.inv(T_0_W)
    # T_G_6 = rvl_manipulator.get_T_G_6()
    T_G_6 = controller.T_G_6
    T_6_G = np.linalg.inv(T_G_6)

    trajectory = []

    T_G_D = poses_G_DD[0]
    T_Arot_A = np.eye(4)
    T_Arot_A[:3, :3] = rot_z(states_rad[0])
    T_6_0_contact = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

    # q_rvl_debug = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # just to make the type checker happy
    # # rvl_manipulator.visualize_vn_model()
    # # rvl_manipulator.visualize_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)
    # rvl_manipulator.visualize_vn_current_state(q_rvl_debug, T_6_0_contact @ T_G_6)


    for i_pose in tqdm(
        range(poses_G_DD.shape[0]),
        desc="Generating trajectories",
        unit="pose",
        disable=not verbose):

        T_G_D = poses_G_DD[i_pose]

        root_waypoints = [] # type: List[Waypoint]

        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(states_rad[0])
        T_6_0_contact = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G
        T_G_W_contact = T_0_W @ T_6_0_contact @ T_G_6

        # rvl_manipulator.set_environment_state(np.rad2deg(states_rad[0]))

        # Get approach path
        T_G_0_via, ik_solutions, paths, _ = rvl_manipulator.approach_path(T_G_W_contact.copy())
        if T_G_0_via.shape[0] < 1 or len(paths) < 1:
            continue
        T_6_0_via = T_G_0_via @ T_6_G[np.newaxis, ...]

        # Adjust IK solutions to ROS and limit ranges
        for ik_sols in ik_solutions:
            ik_sols[:, 0] -= np.pi
            ik_sols[:, 5] -= np.pi
            ik_sols[ik_sols > np.pi] -= (2.0 * np.pi)
            ik_sols[ik_sols < -np.pi] += (2.0 * np.pi)

        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(states_rad[0])
        T_Arot_0 = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A

        T_6_0 = T_Arot_0 @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

        cabinet_model.change_door_angle(np.rad2deg(states_rad[0]))
        cabinet_model.create_mesh()
        cabinet_model.save_mesh()
        controller.add_mesh_to_scene(cabinet_model.mesh_save_path, 'cabinet_model', T_Arot_0, verbose=False)

        # Check for paths in the approach path and add them to the root waypoints
        for path in paths:
            ik0 = np.array(ik_solutions[0][path[0]])
            ik1 = np.array(ik_solutions[1][path[1]])

            for i_sol in range(len(ik_solutions[-1])):
                ik2 = np.array(ik_solutions[-1][i_sol]) # contact point
                if chebyshev_distance(ik2, ik1) <= 0.5 * np.pi and \
                    controller.is_state_valid(ik2.tolist()) and \
                    controller.is_state_valid(ik1.tolist()) and \
                    controller.is_state_valid(ik0.tolist()):
                    wp_root = Waypoint(ik0, T_6_0_via[0])
                    wp_mid = wp_root.add_child(ik1, T_6_0_via[1])
                    wp_mid.add_child(ik2, T_6_0_contact)
                    root_waypoints.append(wp_root)

        # return root_waypoints

        # If no root waypoints were found, skip this pose
        if len(root_waypoints) == 0:
            continue

        # Iterate through the states and try to grow the tree
        for i_state in range(1, num_states):
            state_rad = states_rad[i_state]
            # rvl_manipulator.set_environment_state(np.rad2deg(state_rad))

            T_Arot_A = np.eye(4)
            T_Arot_A[:3, :3] = rot_z(state_rad)
            T_Arot_0 = T_W_0 @ cabinet_model.T_A_W @ T_Arot_A

            T_6_0 = T_Arot_0 @ cabinet_model.T_D_Arot @ T_G_D @ T_6_G

            cabinet_model.change_door_angle(np.rad2deg(state_rad))
            cabinet_model.create_mesh()
            cabinet_model.save_mesh()
            controller.add_mesh_to_scene(cabinet_model.mesh_save_path, 'cabinet_model', T_Arot_0, verbose=False)

            joints: np.ndarray # just to make the type checker happy
            joints, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0)
            if n_sol < 1:
                break
            joints = joints[~np.isnan(joints).any(axis=1)]  # Remove NaN rows
            mask = ~((joints[:, 1] > 0.) & (joints[:, 1] < np.pi))
            joints = joints[mask]
            n_sol = joints.shape[0]

            joints_rvl = joints.copy()
            for i_sol in range(n_sol):
                q = joints[i_sol, :]
                q[0] -= np.pi
                # q[5] -= np.pi
                q[q > np.pi] -= (2.0 * np.pi)
                q[q < -np.pi] += (2.0 * np.pi)

            # For each root waypoint, try to grow the tree
            # If the root is a leaf, we can add the new state directly
            for root in root_waypoints:
                leaves = [root] if root.is_leaf() else get_all_leaves(root)
                for leaf in leaves:
                    best_q = None
                    min_dist = np.inf
                    for i_sol in range(n_sol):
                        q = joints[i_sol, :]
                        q_rvl = joints_rvl[i_sol, :]
                        dist = chebyshev_distance(q, leaf.q)
                        if dist > 0.5 * np.pi:
                            continue
                        # if dist < min_dist and rvl_manipulator.free(q_rvl) and controller.is_state_valid(q.tolist()):
                        if dist < min_dist and controller.is_state_valid(q.tolist()):
                            best_q = q
                            min_dist = dist
                    if best_q is not None:
                        leaf.add_child(best_q, T_6_0)
                        if leaf.children[-1].depth == num_states + 1:
                            # Finish the search - got the desired trajectory
                            leaf_ = copy.deepcopy(leaf.children[-1])
                            while leaf_.parent:
                                trajectory.append(leaf_.q)
                                leaf_ = leaf_.parent
                            trajectory.append(leaf_.q)
                            trajectory.reverse()
                            # rvl_manipulator.visualize_current_state(leaf_.q, T_6_0 @ T_G_6)
                            return np.array(trajectory).reshape(1, -1, 6)
    return np.array([])


def sample_TCP_on_front_surface(s, edge_offset=0.03, step=0.03, height_scale=1.0):
    """
    Sample points on the front surface of the door.

    Args:
        s (np.ndarray): Dimensions of the door [thickness, width, height].
        door_thickness (float): Thickness of the door.
        edge_offset (float): Offset from the edges to avoid sampling too close.
        step (float): Step size for sampling points.

    Returns:
        np.ndarray: Sampled points on the front surface.
    """

    # s = np.array([self.door_thickness, self.s[0], self.s[1]])

    x_ = s[0] * 0.5
    y_, z_ = np.meshgrid(
        -np.arange(edge_offset, s[1] - edge_offset, step),
        # np.arange(-s[2] * (height_scale - 0.5) + edge_offset, s[2] * 0.5 - edge_offset, step), # (height_scale - 0.5) prevents going too low
        np.arange(-s[2] * (height_scale - 0.5) + edge_offset, s[2] * 0.5 - edge_offset, step), # (height_scale - 0.5) prevents going too low
        # np.arange(0.0, s[2] * 0.5 - edge_offset, step),
        indexing='ij'
    )

    front_surface_points = np.stack((np.full_like(y_, x_), y_, z_), axis=-1).reshape(-1, 3)

    R_TCP_A = np.array([[0, 0, 1],
                        [-1, 0, 0],
                        [0, -1, 0]])
    Ry = rot_y(np.deg2rad(-15))
    R_TCP_A = R_TCP_A @ Ry

    # Make all front surface points relative to the S_A
    T_TCP_A_all = np.repeat(np.eye(4).reshape(1, 4, 4), front_surface_points.shape[0], axis=0)
    T_TCP_A_all[:, :3, :3] = R_TCP_A
    T_TCP_A_all[:, :3, 3] = front_surface_points
    print('T_TCP_A_all shape:', T_TCP_A_all.shape)


    T_TCP_A_approach = T_TCP_A_all.copy()
    T_TCP_A_approach[:, :3, 3] += np.array([-5*x_, 0.0, 0.0])  # Move TCP by negative x_ direction

    # Stack the approach points - shape (n, 2, 4, 4)
    T_TCP_A_all = np.stack((T_TCP_A_all, T_TCP_A_approach), axis=1)

    return T_TCP_A_all



def filter_reachable_points(T_TCP_A_all, T_TCP_G, T_Arot_S, robot: UR5Controller, rvl_manipulator: rvlpy.PYDDManipulator, cabinet_mesh: o3d.geometry.TriangleMesh, gripper_mesh: o3d.geometry.TriangleMesh, verbose: bool = True):
    """ Filter reachable points from the sampled TCP positions."""

    # cabinet_mesh_ = copy.deepcopy(cabinet_mesh)
    origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
    # gripper_mesh_ = copy.deepcopy(gripper_mesh)
    T_G_6 = robot.T_G_6
    # rvl_manipulator = self.rvl_manipulator
    # robot = self.robot

    T_Arot_0 = np.linalg.inv(robot.T_0_W) @ T_Arot_S

    feasible_points = []
    for i in tqdm(range(T_TCP_A_all.shape[0]), disable=not verbose, desc="Filtering reachable points", unit="point"):
        T_TCP_A = T_TCP_A_all[i, 0]
        T_TCP_A_approach = T_TCP_A_all[i, 1]

        T_6_0_contact = T_Arot_0 @ T_TCP_A @ np.linalg.inv(T_TCP_G) @ np.linalg.inv(T_G_6)
        T_6_0_approach = T_Arot_0 @ T_TCP_A_approach @ np.linalg.inv(T_TCP_G) @ np.linalg.inv(T_G_6)

        # Check if the approach point is reachable
        joints_contact, n_sol_contact, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0_contact)

        if n_sol_contact < 1:
            continue
        # Filter out NaN and Inf values
        joints_contact = joints_contact[~np.isnan(joints_contact).any(axis=1)]

        joints_approach_all, n_sol_approach, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0_approach)
        if n_sol_approach < 1:
            continue
        # Filter out NaN and Inf values
        joints_approach = joints_approach_all[~np.isnan(joints_approach_all).any(axis=1)]

        joints_contact[:, 0] -= np.pi
        joints_contact[:, 5] -= np.pi
        joints_contact[joints_contact > np.pi] -= 2 * np.pi
        joints_contact[joints_contact < -np.pi] += 2 * np.pi

        mask = ~((joints_contact[:, 1] > 0.) & (joints_contact[:, 1] < np.pi))
        joints_contact = joints_contact[mask]
        n_sol_contact = joints_contact.shape[0]

        joints_approach[:, 0] -= np.pi
        joints_approach[:, 5] -= np.pi
        joints_approach[joints_approach > np.pi] -= 2 * np.pi
        joints_approach[joints_approach < -np.pi] += 2 * np.pi

        mask = ~((joints_approach[:, 1] > 0.) & (joints_approach[:, 1] < np.pi))
        joints_approach = joints_approach[mask]
        n_sol_approach = joints_approach.shape[0]

        # # Visualize the meshes
        # if i == 155:
        #     cabinet_mesh_ = copy.deepcopy(cabinet_mesh)
        #     cabinet_mesh_.transform(T_Arot_0)
        #     T_G_0 = T_Arot_0 @ T_TCP_A @ np.linalg.inv(T_TCP_G)
        #     gripper_mesh_ = copy.deepcopy(gripper_mesh)
        #     gripper_mesh_.transform(T_G_0)
        #     gripper_mesh_.compute_vertex_normals()
        #     T_G_0_approach = T_Arot_0 @ T_TCP_A_approach @ np.linalg.inv(T_TCP_G)
        #     gripper_mesh_approach = copy.deepcopy(gripper_mesh)
        #     gripper_mesh_approach.transform(T_G_0_approach)
        #     gripper_mesh_approach.compute_vertex_normals()
        #     gripper_mesh_approach.paint_uniform_color([0.8, 0.2, 0.2])
        #     tool_rf = copy.deepcopy(origin_rf)
        #     tool_rf.transform(T_6_0_contact)
        #     T_6_0_contact_from_invkin = robot.get_fwd_kinematics_moveit(joints_contact[0])
        #     tool_from_invkin_rf = copy.deepcopy(origin_rf)
        #     tool_from_invkin_rf.transform(T_6_0_contact_from_invkin)
        #     # o3d.visualization.draw_geometries([cabinet_mesh, origin_rf, gripper_mesh, gripper_mesh_approach, tool_rf, tool_from_invkin_rf])
        #     o3d.visualization.draw_geometries([cabinet_mesh_, origin_rf, gripper_mesh_, tool_rf])

        # Check contact joints
        valid_joints_contact = []
        for j in range(joints_contact.shape[0]):
            q = joints_contact[j]
            if np.any(np.isinf(q)):
                continue
            # if robot.is_state_valid(q):
            if robot.is_state_self_collision_free(q): # only self-collision check because the contact is in the cabinet
                valid_joints_contact.append(q)
        if len(valid_joints_contact) < 1:
            continue
        valid_joints_contact = np.array(valid_joints_contact)

        # Check approach joints
        valid_joints_approach = []
        for j in range(joints_approach.shape[0]):
            q = joints_approach[j]
            if np.any(np.isinf(q)):
                continue
            if robot.is_state_valid(q):
            # if robot.is_state_self_collision_free(q): # only self-collision check
                valid_joints_approach.append(q)
        if len(valid_joints_approach) < 1:
            continue
        valid_joints_approach = np.array(valid_joints_approach)

        feasible_points.append((
            T_TCP_A, T_TCP_A_approach, T_6_0_contact, T_6_0_approach,
            valid_joints_contact, valid_joints_approach
        ))


    return feasible_points




def generate_poses_demo():
    # --- Setup fixed transforms ---
    R_TCP_D = np.array([[0, 0, -1],
                        [0, 1, 0],
                        [1, 0, 0]])
    T_TCP_G = np.eye(4)
    T_TCP_G[:3, 3] = np.array([0.155 / 2, 0, 0.102])

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
    tool_finger_distances = [-0.155 / 2., 0., -0.102]  # x, y, z
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
    T_TCP_G[:3, 3] = np.array([0.155 / 2, 0, 0.102])

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
    generate_poses_demo()
    # generate_trajectories_demo()