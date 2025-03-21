#!/usr/bin/python

import rospy
import numpy as np
import sys
import os
from core.util import read_config
from core.transforms import rot_z
from core.meshes import *
import csv
from gazebo_push_open.cabinet_model import Cabinet
import time
import tqdm
from rospkg import RosPack
# np.random.seed(12345)
np.random.seed(69)

def line_seg_to_circle_dist(cir, p1 , p2):
    line_seg = p2-p1
    line_len = np.linalg.norm(line_seg)

    line_unit_vec = line_seg / line_len
    
    # vector from circle to p1
    cir_vec = cir - p1

    # project cir_vec to line
    proj = np.dot(cir_vec, line_unit_vec)
    proj = np.clip(proj, 0, line_len)

    # projected pt
    proj_pt = p1 + proj*line_unit_vec
    
    dist = np.linalg.norm(proj_pt - cir)
    return dist


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


rospy.init_node('node_generate_door_configurations')
rospack = RosPack()

# pkg_path = rospack.get_path('path_planning')
pkg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning'
# Choose epxeriment
exp_name = 'simulation_exp' # simulation_exp, real_exp

if exp_name == 'simulation_exp':
    cfg_path = os.path.join(pkg_path, 'config/config_simulations_axis_left.yaml')
    save_path = os.path.join(pkg_path, 'cabinet_configurations_axis_left.npy')
elif exp_name == 'real_exp':
    cfg_path = os.path.join(pkg_path, 'config/config_multi-c_our_handleless_axis_left_real.yaml')
    save_path = os.path.join(pkg_path, 'cabinet_configurations_axis_left_real.npy')
config = read_config(cfg_path)

HAS_HANDLE = True

NEW_GEN = True

try:
    cabinet_mesh_dirpath = config['cabinet_mesh_save_dir']
    if HAS_HANDLE:
        cabinet_mesh_dirpath = os.path.join(cabinet_mesh_dirpath, 'handle')
    else:
        cabinet_mesh_dirpath = os.path.join(cabinet_mesh_dirpath, 'handleless')

    if not os.path.exists(cabinet_mesh_dirpath):
        os.makedirs(cabinet_mesh_dirpath)
except Exception as e:
    pass


# distance criteria
base_size = close_dist_range = 0.3
far_dist_range = 0.9

# number of doors
n = 1000
poses = np.zeros((n, 8))
cabinet_door_dims = config['cabinet_door_dims'] # w_door, h_door, static_d, d_door
cabinet_pose = config['cabinet_pose']
axis_pos = cabinet_pose['axis_pos']


# Params for height from floor
static_side_width = 0.018
moving_to_static_part_distance = 0.005
distance_from_floor = 0.003 # to ensure that cabinet does not collide with floor

push_latch_mechanism_length = 0.046 + static_side_width*0.5

T_B_S = np.eye(4)
T_B_S[2, 3] = 0.005

# a lot of pts
n_rots = 50
rots = np.linspace(0, axis_pos*np.pi*0.5, n_rots)
Tz_rots = np.zeros((n_rots,4,4))
Tz_rots[:,3,3] = 1.
cs = np.cos(rots)
sn = np.sin(rots)
Tz_rots[:,0,0] = cs
Tz_rots[:,0,1] = -sn
Tz_rots[:,1,0] = sn
Tz_rots[:,1,1] = cs
Tz_rots[:,2,2] = 1.
Tz_rots[:,3,3] = 1.


i = 0
start = time.time()
pbar = tqdm.tqdm(total=n)
while i < n: 
    door_params = [np.random.uniform(cabinet_door_dims['min_width'], cabinet_door_dims['max_width']),
                np.random.uniform(cabinet_door_dims['min_height'], cabinet_door_dims['max_height']),
                cabinet_door_dims['depth'],
                cabinet_door_dims['static_depth']]

    if exp_name == 'simulation_exp':
        cabinet_position = [np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x']),
                            np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y']),
                            static_side_width + moving_to_static_part_distance + distance_from_floor + door_params[1]*0.5 ] # half of door height + offset from bottom
    else:
        # this is to compansate for differences between the cabinet in simulation and the real cabinet
        # the real cabinet has 0.009m offset from bottom, while simulation cabinet has  static_side_width + moving_to_static_part_distance
        door_params[1] = door_params[1] - (static_side_width + moving_to_static_part_distance - 0.009)
        cabinet_position = [np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x']),
                            np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y']),
                            T_B_S[2, 3] + static_side_width + moving_to_static_part_distance + door_params[1]*0.5 ] # half of door height + robot offset from bottom + offset from bottom
                            # 0.009 + 0.005 + door_params[1]*0.5 ] # half of door height + robot offset from bottom + offset from bottom

    rotz_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])


    if exp_name == 'real_exp':
        rotz_deg += 90.
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(cabinet_position)
        # Tz_init = np.eye(4)
        # Tz_init[:3, :3] = rot_z(np.radians(90.))
        # T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rotz_deg))
        T_A_S = T_A_S @ Tz
        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array(door_params), 
                                axis_pos=cabinet_pose['axis_pos'],
                                T_A_S=T_A_S,
                                save_path=config['cabinet_urdf_save_path'])
        
        T_pt1_A = np.eye(4)
        T_pt1_A[:3, 3] = np.array([0., 0., 0.])
        T_pt2_A = np.eye(4)
        T_pt2_A[:3, 3] = np.array([cabinet_model.static_d, 0., 0.])
        T_pt3_A = np.eye(4)
        T_pt3_A[:3, 3] = np.array([cabinet_model.static_d, -cabinet_model.w_door, 0.])
        T_pt4_A = np.eye(4)
        T_pt4_A[:3, 3] = np.array([0., -cabinet_model.w_door, 0.])

        # debug
        T_pt2_A_ = np.eye(4)
        T_pt2_A_[:3, 3] = np.array([-0.009, -cabinet_model.w_door, cabinet_model.h_door*0.5])

        T_pt1_S = T_A_S @ T_pt1_A
        T_pt2_S = T_A_S @ T_pt2_A
        T_pt3_S = T_A_S @ T_pt3_A
        T_pt4_S = T_A_S @ T_pt4_A
        
        angle_deg = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length/door_params[0]))
        
        pts_crit = T_pt1_S[0, 3] > -0.35 and T_pt2_S[0, 3] > -0.35 and T_pt3_S[0, 3] < 0.35 and T_pt4_S[0, 3] < 0.35

        dist_crit = True
        if NEW_GEN:
            T_D_A = cabinet_model.T_D_A.copy()
            T_D_A[0,3] = 0. # the axis and the free point for rotation are aligned on the door axis

            T_D_S_rots = cabinet_model.T_A_S[np.newaxis,...] @ Tz_rots @ T_D_A[np.newaxis,...]

            # distance of line segment (doors) to base pt
            # dist_door_line_0 = line_seg_to_circle_dist(np.zeros((2,)), T_A_S[:2,3], T_D_S_0[:2,3])
            # dist_door_line_45 = line_seg_to_circle_dist(np.zeros((2,)), T_A_S[:2,3], T_D_S_45[:2,3])
            # dist_door_line_90 = line_seg_to_circle_dist(np.zeros((2,)), T_A_S[:2,3], T_D_S_90[:2,3])

            closest_dists = line_seg_to_circle_dist_all(np.zeros((2,)), T_A_S[:2,3], T_D_S_rots[:,:2,3])
            T_D_S_dists = np.linalg.norm(T_D_S_rots[:, :2, 3], axis=1)

            dist_crit = np.all((closest_dists > close_dist_range) & (T_D_S_dists < far_dist_range))


        if pts_crit and dist_crit:
            # poses[i, :] = np.array([door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg, axis_pos])
            poses[i, :] = np.array([door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg, angle_deg, axis_pos])
            i += 1
            pbar.update(1)
    
    elif exp_name == 'simulation_exp':
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(cabinet_position)
        # T_A_S[2, 3] += T_B_S[2, 3]
        Tz_init = np.eye(4)
        Tz_init[:3, :3] = rot_z(np.radians(90.))
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rotz_deg))
        T_A_S = T_A_S @ Tz
        
        cabinet_urdf_path = os.path.join(cabinet_mesh_dirpath, 'cabinet_whole_%d.urdf' % i)
        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array(door_params), 
                                axis_pos=cabinet_pose['axis_pos'],
                                T_A_S=T_A_S,
                                save_path=cabinet_urdf_path,
                                has_handle=HAS_HANDLE)
        cabinet_static_ply_path = os.path.join(cabinet_mesh_dirpath, 'cabinet_static_%d.ply' % i)
        cabinet_static_dae_path = os.path.join(cabinet_mesh_dirpath, 'cabinet_static_%d.dae' % i)
        cabinet_whole_ply_path = os.path.join(cabinet_mesh_dirpath, 'cabinet_whole_%d.ply' % i)
        cabinet_whole_dae_path = os.path.join(cabinet_mesh_dirpath, 'cabinet_whole_%d.dae' % i)
        cabinet_panel_ply_path = os.path.join(cabinet_mesh_dirpath, 'cabinet_panel_%d.ply' % i)

        # cabinet_model.save_mesh_without_doors(cabinet_static_ply_path)
        # cabinet_model.save_door_panel_mesh(cabinet_panel_ply_path)
        # cabinet_model.save_mesh(cabinet_whole_ply_path)
        if HAS_HANDLE:
            # convert_to_dae(cabinet_static_ply_path, cabinet_static_dae_path, object_name='cabinet')
            # fix_dae_up_axis(cabinet_static_dae_path, axis_up='Z_UP')
            # set_model_name_in_dae(cabinet_static_dae_path, model_name='cabinet')
            
            # convert_to_dae(cabinet_whole_ply_path, cabinet_whole_dae_path, object_name='cabinet')
            # fix_dae_up_axis(cabinet_whole_dae_path, axis_up='Z_UP')
            # set_model_name_in_dae(cabinet_whole_dae_path, model_name='cabinet')
            # convert_ply_to_obj(cabinet_whole_ply_path, object_name='cabinet')
            pass

        # angle_deg = axis_pos * np.rad2deg(np.arctan(push_latch_mechanism_length/door_params[0]))
        angle_deg = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length/door_params[0]))
        
        T_D_S_0 = cabinet_model.T_A_S @ cabinet_model.T_D_A

        if NEW_GEN:
            T_D_A = cabinet_model.T_D_A.copy()
            T_D_A[0,3] = 0. # the axis and the free point for rotation are aligned on the door axis

            T_D_S_rots = cabinet_model.T_A_S[np.newaxis,...] @ Tz_rots @ T_D_A[np.newaxis,...]

            # distance of line segment (doors) to base pt
            # dist_door_line_0 = line_seg_to_circle_dist(np.zeros((2,)), T_A_S[:2,3], T_D_S_0[:2,3])
            # dist_door_line_45 = line_seg_to_circle_dist(np.zeros((2,)), T_A_S[:2,3], T_D_S_45[:2,3])
            # dist_door_line_90 = line_seg_to_circle_dist(np.zeros((2,)), T_A_S[:2,3], T_D_S_90[:2,3])

            closest_dists = line_seg_to_circle_dist_all(np.zeros((2,)), T_A_S[:2,3], T_D_S_rots[:,:2,3])
            T_D_S_dists = np.linalg.norm(T_D_S_rots[:, :2, 3], axis=1)

            dist_crit = np.all((closest_dists > close_dist_range) & (T_D_S_dists < far_dist_range))

        else:
            Tz_45 = np.eye(4)
            Tz_45[:3,:3] = rot_z(np.radians(axis_pos * 45.))
            T_D_S_45 = cabinet_model.T_A_S @ Tz_45 @ cabinet_model.T_D_A 
            Tz_90 = np.eye(4)
            Tz_90[:3,:3] = rot_z(np.radians(axis_pos * 90.))
            T_D_S_90 = cabinet_model.T_A_S @ Tz_90 @ cabinet_model.T_D_A
            Tz_state_angle = np.eye(4)
            Tz_state_angle[:3,:3] = rot_z(np.radians(angle_deg))
            T_D_S_angle =cabinet_model.T_A_S @ Tz_state_angle @ cabinet_model.T_D_A


            dist_0 = np.linalg.norm(T_D_S_0[:2, 3])
            dist_45 = np.linalg.norm(T_D_S_45[:2, 3])
            dist_90 = np.linalg.norm(T_D_S_90[:2, 3])
            dist_state_angle = np.linalg.norm(T_D_S_angle[:2, 3])
            dist_axis = np.linalg.norm(T_A_S[:2, 3])

            dist_crit = close_dist_range < dist_0 < far_dist_range and close_dist_range < dist_45 < far_dist_range and close_dist_range < dist_90 < far_dist_range and close_dist_range < dist_state_angle < far_dist_range and dist_axis > close_dist_range
        
        # if dist_crit != dist_crit_all:
        #     print('aaaa')

        dist_axis = np.linalg.norm(T_A_S[:2, 3])
        close_axis_dist_from_base = dist_axis > base_size 

        is_properly_rotated = np.dot(T_D_S_0[:3, 3], T_D_S_0[:3, 2]) > 0.

        if dist_crit and close_axis_dist_from_base and is_properly_rotated and T_D_S_0[1, 3] > 0.:
            poses[i, :] = np.array([door_params[0], door_params[1], cabinet_position[0], cabinet_position[1], cabinet_position[2], rotz_deg, angle_deg, axis_pos])
            i += 1
            pbar.update(1)

print(time.time() - start)
pbar.close()
np.save(save_path, poses)

rospy.signal_shutdown('shutdown')