import numpy as np
import os
from core.transforms import *
from tqdm import tqdm
from gazebo_push_open.cabinet_model2 import Cabinet2
from core.real_ur5_controller import UR5Controller
from push_force_trajectories import *
from force_utils import *
import pickle
import open3d as o3d
import RVLPYDDManipulator as rvlpy


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


def line_seg_to_circle_dist_all(cir, p1, p2s):
    """
    Computes the minimum distances from a point (cir) to multiple line segments.
    Each segment is defined by a fixed starting point p1 and an array of endpoints p2s.

    Parameters:
      cir: array-like of shape (2,), the point (e.g., circle center) as (x, y)
      p1: array-like of shape (2,), the fixed endpoint (e.g., door hinge)
      p2s: array-like of shape (N,2), the other endpoints for each rotated door position

    Returns:
      A NumPy array of shape (N,) containing the Euclidean distances from cir to the closest 
      point on each segment.
    """
    # Ensure inputs are NumPy arrays with float type
    p1 = np.array(p1, dtype=float)
    cir = np.array(cir, dtype=float)
    p2s = np.array(p2s, dtype=float)  # shape: (N, 2)

    # Compute the segment vectors for each rotated door (p2s - p1)
    seg = p2s - p1                # shape: (N,2)
    seg_len_sq = np.sum(seg**2, axis=1)  # shape: (N,)

    # Vector from p1 to the circle center (broadcasted)
    cir_vec = cir - p1            # shape: (2,)

    # Dot product between cir_vec and each segment vector
    dot_val = np.sum(seg * cir_vec, axis=1)  # shape: (N,)

    # Compute projection factor for each segment and clamp between 0 and 1.
    # If a segment is degenerate (length zero), t is set to 0.
    t = np.where(seg_len_sq > 0, np.clip(dot_val / seg_len_sq, 0, 1), 0)

    # Compute the projected points along each segment
    proj_pts = p1 + t[:, None] * seg  # shape: (N,2)

    # Compute distances from the circle center to each projected point
    dist = np.linalg.norm(proj_pts - cir, axis=1)
    return dist


rospy.init_node('generate_gt_poses', anonymous=True)

n = 100000
n_saved = 10
width = 0.395
height = 0.495
door_thickness = 0.018
static_depth = 0.4  # Static depth of the cabinet
axis_pos = -1
# push_latch_mechanism_length = 0.046 + 0.018 * 0.5
push_latch_mechanism_length = 0.0445
latch_offset = 0.023
state_angle = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length/(width-latch_offset)))
state_offset = axis_pos * 5.0

gripper_mesh = o3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply')

IS_OFFLINE = False  # Set to True for offline generation
# num_poses_per_cabinet = 3 if IS_OFFLINE else 1

if IS_OFFLINE:
    np.random.seed(12345)
else:
    np.random.seed(12346)

num_poses_per_cabinet = 1
pose_angles = np.array(np.deg2rad(
    [0.0, -15.0, 15.0])) if IS_OFFLINE else np.array([0.0])
Tz_poses = np.zeros((pose_angles.shape[0], 4, 4))
Tz_poses[:, :3, :3] = rotz_multiple(pose_angles)
Tz_poses[:, 3, 3] = 1

opening_angle = state_angle + state_offset if IS_OFFLINE else -45.0
num_traj_pts = 3 if IS_OFFLINE else 10

base_size = close_dist_range = 0.3
far_dist_range = 0.9

T_A_W = np.eye(4)
T_A_W[:3, :3] = np.array([[0, -1, 0],
                          [1, 0, 0],
                          [0, 0, 1]])

# Random generate cabinet position
cabinet_positions = []
z = height*0.5 + 0.01  # 0.01 is the offset of the cabinet from the ground
table_width = 0.7
table_width_2 = table_width * 0.5
for i in range(n):
    x = np.random.uniform(-table_width_2, 0.0)
    y = np.random.uniform(0.4, 0.8)
    cabinet_positions.append([x, y, z])
cabinet_positions = np.array(cabinet_positions)

# z_rotations = np.random.uniform(0, 0, size=(n,))
# z_rotations = np.random.uniform(-np.pi*0.25, np.pi*0.25, size=(n,))
range_z_rotations = np.pi * 1 / 3
z_rotations = np.random.uniform(-range_z_rotations,
                                range_z_rotations, size=(n,))
Tz = np.zeros((n, 4, 4))
Tz[:, :3, :3] = rotz_multiple(z_rotations)
Tz[:, 3, 3] = 1

T_A_W_cabinets = np.zeros((n, 4, 4))
T_A_W_cabinets[:] = T_A_W[np.newaxis, ...] @ Tz
T_A_W_cabinets[:, :3, 3] = cabinet_positions
T_A_W_cabinets[:, 3, 3] = 1

# n_rots = 2 if IS_OFFLINE else 50
n_rots = 50
# rots = np.linspace(0, np.deg2rad(opening_angle), n_rots)
rots = np.linspace(0, np.deg2rad(-90.), n_rots)
Tz_rots = np.zeros((n_rots, 4, 4))
Tz_rots[:, :3, :3] = rotz_multiple(rots)
Tz_rots[:, 3, 3] = 1

# cabinet_model_init = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]),
#                             r=np.array([0., axis_pos*width*0.5]),
#                             axis_pos=axis_pos,
#                             T_A_S=np.eye(4),
#                             save_path=None)
s = np.array([door_thickness, width, height, static_depth])
r = np.array([axis_pos * 0.5 * s[0], -0.5*s[1]])
cabinet_model_init = Cabinet2(s=s,
                              r=r,
                              axis_pos=axis_pos,
                              T_A_W=np.eye(4),
                              save_path=None)
T_D_A = cabinet_model_init.T_D_Arot.copy()  # When closed

tactile_sensor_depth = 0.006
tactile_sensor_width = 0.023

grasp_one_finger = True  # True for one finger, False for two fingers

# tx_E = 0.155
tx_E = 0.15 * 0.5 - tactile_sensor_depth
tz_E = 0.278
# tz_G = 0.1045 if grasp_one_finger else 0.100
tx_G = 0.0721
tz_G = 0.1041 if grasp_one_finger else 0.100

T_TCP_G = np.eye(4)
T_TCP_G[:3, 3] = np.array([-tx_G, 0, tz_G])
# if grasp_one_finger:
#     Tz_ = np.eye(4)
#     Tz_[:3, :3] = rot_z(np.pi)
#     T_TCP_G = Tz_ @ T_TCP_G  # Rotate around Z axis for one finger grasp
R_TCP_D = np.array([[0, 0, -1],
                    [0, -1, 0],
                    [-1, 0, 0]])

T_Arot_A = np.eye(4)
T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))

# # Generate tool poses for front surface touches - deprecated
# s = np.array([door_thickness, width, height])
# T_TCP_A_all = sample_TCP_on_front_surface(s, edge_offset=0.03, step=0.03)
# T_TCP_Arot_all = np.linalg.inv(T_Arot_A)[np.newaxis, ...] @ T_TCP_A_all.reshape(-1, 4, 4)

robot = UR5Controller()
robot.T_0_W = np.eye(4)
rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

rvl_manipulator = rvlpy.PYDDManipulator()
rvl_manipulator.create(rvl_cfg)
rvl_manipulator.set_robot_pose(robot.T_0_W)

rvl_touch_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Touch_Cupec.cfg'
py_touch = rvl_manipulator.py_touch
py_touch.create(rvl_touch_cfg)
camera_fu = 597.9033203125
camera_fv = 598.47998046875
camera_uc = 323.8436584472656
camera_vc = 236.32774353027344
camera_w = 640
camera_h = 480
py_touch.set_camera_params(
    camera_fu, camera_fv, camera_uc, camera_vc, camera_w, camera_h)
touch_a = static_depth
touch_b = 0.0
touch_c = 0.005
T_C_E = np.eye(4)  # This is only a placeholder

rvl_manipulator.set_door_model_params(
    s[0],
    s[1],
    s[2],
    r[0],
    r[1],
    -1,
    s[0],
    touch_c)
rvl_manipulator.set_door_pose(np.eye(4))

base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'
cabinet_mesh_path = os.path.join(base_dir, 'cabinet_model/cabinet_mesh.ply')
cabinet_static_mesh_path = os.path.join(
    base_dir, 'cabinet_model/cabinet_static.ply')
cabinet_panel_mesh_path = os.path.join(
    base_dir, 'cabinet_model/cabinet_panel.ply')

poses_filename = os.path.join(
    base_dir, 'gt_cabinets', 'offline_cabinet_gt_poses.pkl' if IS_OFFLINE else 'cabinet_gt_poses.pkl')
trajectories_filename = os.path.join(
    base_dir, 'gt_cabinets', 'offline_cabinet_gt_trajectories.pkl' if IS_OFFLINE else 'cabinet_gt_trajectories.pkl')

robot.T_G_6 = np.load(os.path.join(base_dir, 'T_G_6.npy'))

T_A_W_saved = []
trajectories_per_pose = []
# count_pts = 0
# count_dist = 0

# gripper_mesh_path = os.path.join('/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply')
# gripper_mesh = o3d.io.read_triangle_mesh(gripper_mesh_path)

T_G_DD_all = generate_tool_line_poses(
    height, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)

# DEBUG Visualization
plate_mesh = copy.deepcopy(cabinet_model_init.plate_mesh)
T_D_O = cabinet_model_init.T_A_O @ cabinet_model_init.T_D_A
plate_mesh.transform(np.linalg.inv(T_D_O))
plate_mesh.compute_vertex_normals()
visualize_poses(gripper_mesh, plate_mesh, T_G_DD_all[6000:7000], T_TCP_G)

# COLLISION WITH DOORS
LOAD_COLLISION_POSES = True  # Set to True to load precomputed collision-free poses
T_G_DD_all_colfree_path = os.path.join(base_dir, 'T_G_DD_all_colfree_gt.npy')
# Collision detection
if not LOAD_COLLISION_POSES:
    cabinet_model_init.change_door_angle(state_angle)
    cabinet_model_init.create_mesh()

    # T_O_D = np.linalg.inv(cabinet_model_init.T_D_A_init) @ cabinet_model_init.T_A_O
    T_O_D = np.linalg.inv(cabinet_model_init.T_D_A) @ np.linalg.inv(cabinet_model_init.T_A_O)

    T_G_DD_ = T_G_DD_all[6000]
    # visualize_poses(self.gripper_mesh, plate_mesh, self.T_G_DD_all_full[0:2000])

    gripper_mesh_ = copy.deepcopy(gripper_mesh)
    gripper_mesh_.transform(T_G_DD_)
    gripper_mesh_.compute_vertex_normals()

    origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    plate_mesh = copy.deepcopy(cabinet_model_init.plate_mesh)
    plate_mesh.transform(T_O_D)
    plate_mesh.compute_vertex_normals()
    static_mesh = copy.deepcopy(cabinet_model_init.static_mesh)
    static_mesh.transform(T_O_D)
    static_mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries(
        [origin_rf, plate_mesh, static_mesh, gripper_mesh_])

    collisions = collision_detection_fcl(
        gripper_mesh, T_G_DD_all, plate_mesh, T_TCP_G=T_TCP_G, static_mesh=static_mesh)
    collisions = np.array(collisions, dtype=bool)
    collisions = np.invert(collisions)

    T_G_DD_all_colfree = T_G_DD_all[collisions, ...]
    np.save(T_G_DD_all_colfree_path, T_G_DD_all_colfree)
    T_TCP_D_colfree = T_G_DD_all_colfree.reshape(
        -1, 4, 4) @ T_TCP_G[np.newaxis, ...]
else:
    T_G_DD_all_colfree = np.load(T_G_DD_all_colfree_path)
    T_TCP_D_colfree = T_G_DD_all_colfree.reshape(
        -1, 4, 4) @ T_TCP_G[np.newaxis, ...]


pbar = tqdm(total=n_saved, desc="Valid Cabinet Poses", unit="cabinet")
for i_cabinet in range(n):
    if len(T_A_W_saved) >= n_saved:
        break

    T_A_W_cabinet_init = T_A_W_cabinets[i_cabinet]
    pose_valid = True
    T_A_W_poses_ = []
    T_A_W_trajectories = []
    pbar.set_postfix({"Attempts": i_cabinet})

    for j in range(num_poses_per_cabinet):

        T_A_W_cabinet = T_A_W_cabinet_init @ Tz_poses[j]

        T_pt1_A = np.eye(4)
        T_pt1_A[:3, 3] = np.array([0., 0., 0.])
        T_pt2_A = np.eye(4)
        T_pt2_A[:3, 3] = np.array([static_depth, 0., 0.])
        T_pt3_A = np.eye(4)
        T_pt3_A[:3, 3] = np.array([static_depth, -width, 0.])
        T_pt4_A = np.eye(4)
        T_pt4_A[:3, 3] = np.array([0., -width, 0.])

        T_pt1_W = T_A_W_cabinet @ T_pt1_A
        T_pt2_W = T_A_W_cabinet @ T_pt2_A
        T_pt3_W = T_A_W_cabinet @ T_pt3_A
        T_pt4_W = T_A_W_cabinet @ T_pt4_A

        pts_crit = T_pt1_W[0, 3] > -table_width_2 and \
            T_pt2_W[0, 3] > -table_width_2 and \
            T_pt3_W[0, 3] < table_width_2 and \
            T_pt4_W[0, 3] < table_width_2
        if not pts_crit:
            pose_valid = False
            break

        T_D_W_rots = T_A_W_cabinet[np.newaxis, ...] @ Tz_rots @ T_D_A[np.newaxis, ...]
        closest_dists = line_seg_to_circle_dist_all(np.zeros((2,)), T_A_W_cabinet[:2, 3], T_D_W_rots[:, :2, 3])
        T_D_W_dists = np.linalg.norm(T_D_W_rots[:, :2, 3], axis=1)
        dist_crit = np.all((closest_dists > close_dist_range)
                           & (T_D_W_dists < far_dist_range))
        if not dist_crit:
            pose_valid = False
            break

        # s = np.array([door_thickness, width, height, static_depth])
        cabinet_model = Cabinet2(s=s,
                                 r=r,
                                 axis_pos=axis_pos,
                                 T_A_W=T_A_W_cabinet,
                                 save_path=None)
        cabinet_model.mesh_save_path = cabinet_mesh_path
        cabinet_model.change_door_angle(state_angle)
        cabinet_mesh = cabinet_model.create_mesh()
        cabinet_mesh.transform(np.linalg.inv(cabinet_model.T_A_O))

        T_Arot_W = cabinet_model.T_A_W @ T_Arot_A

        # DEPRECATED
        # if IS_OFFLINE:
        #     feasible_points = filter_reachable_points(T_TCP_A_all, T_TCP_G, T_Arot_W, robot, rvl_manipulator, cabinet_mesh, gripper_mesh, verbose=False)
        #     if len(feasible_points) < 3:
        #         pose_valid = False
        #         break

        # # Visualize the cabinet
        # # T_O_W = T_A_W_cabinet @ np.linalg.inv(cabinet_model.T_A_O)
        # T_O_W = T_A_W_cabinet @ np.linalg.inv(cabinet_model.T_A_O)
        # mesh = cabinet_model.create_mesh()
        # origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # mesh.paint_uniform_color([0.5, 0.5, 0.5])
        # mesh_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # mesh_rf.transform(T_O_W)
        # mesh.transform(T_O_W)
        # mesh.compute_vertex_normals()
        # o3d.visualization.draw_geometries([mesh, origin_rf, mesh_rf], window_name='Cabinet', width=800, height=600, left=50, top=50, mesh_show_back_face=True)

        py_touch.set_session_params(s[0], s[1], s[2], r[0], r[1], touch_a, touch_b, touch_c, state_angle, T_C_E,
                                    s[0], s[1], s[2], r[0], r[1], state_angle, T_Arot_W)
        rvl_manipulator.set_environment_from_touch_gt()
        T_D_S = cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot
        rvl_manipulator.set_pose_DD_S(T_D_S)
        # rvl_manipulator.visualize_vn_model()

        # Check if the opening path exists
        if IS_OFFLINE:
            trajectories = generate_one_trajectory_and_approach(
                T_G_DD_all_colfree,
                num_traj_pts,
                state_angle,
                opening_angle,
                cabinet_model,
                rvl_manipulator,
                robot.T_0_W,
                robot,
                verbose=False
            )
            _, trajectories_ = generate_trajectories_and_approach3(
                T_G_DD_all_colfree,
                num_traj_pts,
                state_angle,
                opening_angle,
                cabinet_model,
                rvl_manipulator,
                robot.T_0_W,
                robot)
        else:
            _, trajectories = generate_trajectories_and_approach3(
                T_G_DD_all_colfree,
                num_traj_pts,
                state_angle,
                opening_angle,
                cabinet_model,
                rvl_manipulator,
                robot.T_0_W,
                robot)

        if trajectories.shape[0] < 1:
            # continue
            # print(f"Cabinet {i_cabinet}, pose {j} has no feasible trajectories. Skipping.")
            pose_valid = False
            break

        print(trajectories.shape[0], "trajectories found for cabinet ", len(T_A_W_poses_))
        # Save the pose and trajectories
        T_A_W_poses_.append(T_A_W_cabinet)
        T_A_W_trajectories.append(trajectories)

    if len(T_A_W_poses_) == num_poses_per_cabinet:
        trajectories_per_pose.append(T_A_W_trajectories)
        T_A_W_saved.append(T_A_W_poses_)
        # print(f"Cabinet {i_cabinet} saved. Total cabinets: {len(T_A_W_saved)}")
        pbar.update(1)
        # with open(poses_filename, 'wb') as f:
        #     pickle.dump(T_A_W_saved, f)
        # # print(f"Saved poses to {poses_filename}")
        # with open(trajectories_filename, 'wb') as f:
        #     pickle.dump(trajectories_per_pose, f)

pbar.close()

print(f"Total cabinets: {len(T_A_W_saved)}")
T_A_W_saved = np.array(T_A_W_saved)

# np.save(poses_filename, T_A_W_saved)
with open(poses_filename, 'wb') as f:
    pickle.dump(T_A_W_saved, f)
print(f"Saved poses to {poses_filename}")
with open(trajectories_filename, 'wb') as f:
    pickle.dump(trajectories_per_pose, f)
