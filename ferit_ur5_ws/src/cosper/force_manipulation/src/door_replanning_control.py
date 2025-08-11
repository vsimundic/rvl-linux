#!/usr/bin/python

import rospy
import os, sys
import numpy as np
import json
import threading
from tf2_ros import Buffer, TransformListener
from core.real_ur5_controller import UR5Controller
from core.image_process import OneShotImageCapture
from core.util import furthest_point_sampling, get_nearest_joints, get_nearest_joints_pair_indices
sys.path.append(os.path.join(os.path.dirname(__file__), '../../door_detection/src'))
from door_state_detector import DoorStateDetector
from push_force_trajectories import *
from force_utils import *
from gazebo_push_open.cabinet_model2 import Cabinet2
import RVLPYDDManipulator as rvlpy
import pickle
import open3d as o3d
from enum import Enum
from typing import List, Tuple
from collections import defaultdict
import matplotlib.pyplot as plt
import csv

np.random.seed(12345)

class TouchType(Enum):
    UNWANTED_TOUCH = 0
    MISS = 1
    WANTED_TOUCH = 2

class RVLTool:
    def __init__(self, a, b, c, d, h, tx, tz, rot_z_angle=-np.pi*0.25, rot_y_angle=np.deg2rad(3.0)):
        self.a = a  # depth of the tool box
        self.b = b  # width of bottom tool edge
        self.c = c  # depth of the tool edge
        self.d = d  # width of the top tool edge
        self.h = h  # height of the tool

        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(rot_z_angle)

        Ty = np.eye(4)
        Ty[:3, :3] = rot_y(rot_y_angle)

        self.T_TCP_6 = np.eye(4)  
        self.T_TCP_6[:3, 3] = np.array([tx, 0, tz])
        self.T_TCP_6 = Tz @ self.T_TCP_6
        self.T_TCP_6 = self.T_TCP_6 @ Ty
        
        # self.T_TCP_6 = Tz.copy()
        # self.T_TCP_6[:3, 3] += -self.T_TCP_6[:3, 0] * tx * 0.5
        # self.T_TCP_6[:3, 3] += -self.T_TCP_6[:3, 2] * tz * 0.5
        # self.T_TCP_6 = self.T_TCP_6 @ Ty

        self.T_tool_TCP = np.eye(4)
        self.T_tool_TCP[:3, 3] = np.array([-a*0.5, 0, -h*0.5])

        self.T_tool_6 = self.T_TCP_6 @ self.T_tool_TCP

        # self.T_tool_6 = np.eye(4) # tool box center pose in the flange frame
        # self.T_tool_6[:3, 3] = np.array([0, 0, tz - h * 0.5])  # position of the tool box center in the flange frame
        # self.T_tool_6 = self.T_tool_6 @ Tz  # rotate the tool box
        # self.T_tool_6[:3, 3] += self.T_tool_6[:3, 0] * -(tx + a * 0.5)  # move the tool box to the correct position
        # self.T_tool_6 = self.T_tool_6 @ Ty  # rotate the tool box
        # self.T_tool_6[:3, 3] += -self.T_tool_6[:3, 0] * 0.0003
        # self.T_tool_6[:3, 3] += self.T_tool_6[:3, 2] * 0.00025


class DoorReplanningFSM:
    def __init__(self, gt_idx=0, IS_OFFLINE=False, CORRECT_ON_TOUCH=False, LOAD_SESSION=False):
        rospy.init_node('door_replanning_fsm_node')

        self.timestamp = rospy.get_time()
        # Initialize ROS utilities
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.IS_OFFLINE = IS_OFFLINE
        self.CORRECT_ON_TOUCH = CORRECT_ON_TOUCH
        self.LOAD_SESSION = LOAD_SESSION
        self.grasp_one_finger = True  # True for one finger, False for two fingers

        # Paths and configurations
        self.base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'

        self.rng_state_path = os.path.join(self.base_dir, 'rng_state.pkl')

        if not os.path.exists(self.rng_state_path):
            with open(self.rng_state_path, 'wb') as f:
                pickle.dump(np.random.get_state(), f)

        self.gt_poses_path = os.path.join(self.base_dir, 'gt_cabinets/{}.pkl'.format('cabinet_gt_poses'))
        self.offline_gt_poses_path = os.path.join(self.base_dir, 'gt_cabinets/{}.pkl'.format('offline_cabinet_gt_poses'))
        self.gt_trajectories_path = os.path.join(self.base_dir, 'gt_cabinets/cabinet_gt_trajectories.pkl')

        with open(self.gt_poses_path, 'rb') as f:
            self.gt_poses = pickle.load(f)
        with open(self.offline_gt_poses_path, 'rb') as f:
            self.offline_gt_poses = pickle.load(f)
        with open(self.gt_trajectories_path, 'rb') as f:
            self.trajectories_per_pose = pickle.load(f)
        
        self.selected_poses = self.gt_poses
        
        self.gt_idx = gt_idx
        self.offline_gt_idx = 0
        self.scene_idx = 0
        self.session_idx = 0
        self.idx_to_save = 0
        self.touch_session_set = False

        self.door_detector_config_path = '/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg'
        self.rvl_ddmanipulator_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
        self.rvl_touch_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Touch_Cupec.cfg'
        self.rvl_manipulator = rvlpy.PYDDManipulator()
        self.rvl_manipulator.create(self.rvl_ddmanipulator_cfg)
        # self.rvl_manipulator.set_robot_pose(self.robot.T_0_W)
        # Touch initialization
        self.py_touch = self.rvl_manipulator.py_touch
        self.py_touch.create(self.rvl_touch_cfg)
        self.touch_is_init = True

        self.gripper_mesh_path = os.path.join('/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply')
        self.gripper_mesh = o3d.io.read_triangle_mesh(self.gripper_mesh_path)

        self.door_thickness = 0.018
        self.static_depth = 0.4
        self.max_width = 0.6
        self.max_height = 0.8
        self.gt_height = 0.495
        self.gt_width = 0.395

        self.tactile_sensor_depth = 0.006
        self.tactile_sensor_width = 0.023

        # Cabinet model
        self.cabinet_model = None
        self.state_angle = None
        self.opening_angle = -45.0
        self.push_latch_mechanism_length = 0.0445
        self.latch_offset = 0.023
        self.T_6_0_capture = np.eye(4)

        self.T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')
        # Unscale the T_C_6 matrix
        R_C_6 = self.T_C_6[:3, :3]
        I_ = R_C_6 @ R_C_6.T
        mean_trace = np.trace(I_) / 3.0
        self.scale_factor = np.sqrt(mean_trace)
        self.T_C_6[:3, :3] /= self.scale_factor
        self.T_C_6_init = self.T_C_6.copy()

        # Initialize robot and manipulator
        self.robot = UR5Controller()
        # self.robot = UR5Commander()
        self.robot.T_C_6 = self.T_C_6

        self.robot.T_G_6 = np.load(os.path.join(self.base_dir, 'T_G_6.npy'))
        # self.robot.T_G_6[:2, 3] = 0.0
        # self.robot.T_G_6 = self.rvl_manipulator.get_T_G_6().astype(np.float64)
        # Tz = np.eye(4)
        # Tz[:3, :3] = rot_z(np.pi)
        # self.robot.T_G_6 = Tz @ self.robot.T_G_6
        # np.save(os.path.join(self.base_dir, 'T_G_6.npy'), Tz @ self.robot.T_G_6)

        # Tool parameters
        # tx_E = 0.155 - self.tactile_sensor_depth
        tx_E = 0.15 * 0.5 - self.tactile_sensor_depth
        tz_E = 0.278
        
        # Params from inspection in blender for the mesh
        tx_G = 0.0721
        tz_G = 0.1041 if self.grasp_one_finger else 0.100

        a_tool = 0.0205
        b_tool = 0.032
        c_tool = 0.011
        d_tool = 0.026
        h_tool = 0.023
        rotz_angle = -np.pi * 0.25 # to get the tcp on the one finger instead of two
        rvl_tool = RVLTool(a_tool, b_tool, c_tool, d_tool, h_tool, -tx_E, tz_E, rot_z_angle=rotz_angle)
        T_tool_6 = rvl_tool.T_tool_6 #type: np.ndarray
        self.T_TCP_6 = rvl_tool.T_TCP_6 #type: np.ndarray
        np.save(os.path.join(self.base_dir, 'T_tool_6.npy'), rvl_tool.T_tool_6)

        self.T_0_W = np.eye(4)

        # self.T_TCP_G = np.linalg.inv(self.robot.T_G_6) @ T_TCP_6
        self.T_TCP_G = np.eye(4)
        self.T_TCP_G[:3, 3] = np.array([-tx_G, 0, tz_G])

        # self.robot.T_G_6 = T_TCP_6 @ np.linalg.inv(self.T_TCP_G)
        # np.save(os.path.join(self.base_dir, 'T_G_6.npy'), self.robot.T_G_6)

        # if self.grasp_one_finger:
        #     Tz_ = np.eye(4)
        #     Tz_[:3, :3] = rot_z(np.pi)
        #     self.T_TCP_G = Tz_ @ self.T_TCP_G  # Rotate around Z axis for one finger grasp
        self.R_TCP_D = np.array([[0, 0, -1],
                                [0, -1, 0],
                                [-1, 0, 0]])

        self.T_G_DD_all_full = generate_tool_line_poses(self.max_height, self.T_TCP_G, self.R_TCP_D).reshape(-1, 4, 4)

        LOAD_COLLISION_POSES = True
        T_G_DD_all_colfree_path = os.path.join(self.base_dir, 'T_G_DD_all_colfree.npy')
        if not os.path.exists(T_G_DD_all_colfree_path):
            LOAD_COLLISION_POSES = False
        # Collision detection
        if not LOAD_COLLISION_POSES:
            state_angle = -1 * np.rad2deg(np.arcsin(self.push_latch_mechanism_length / (self.max_width - self.latch_offset)))
            cabinet_model_init = Cabinet2(s=np.array([self.door_thickness, self.max_width, self.max_height, self.static_depth]),
                                    r=np.array([-self.door_thickness*0.5, -self.max_width*0.5]),
                                    axis_pos=-1,
                                    T_A_W=np.eye(4),
                                    save_path=None)
            cabinet_model_init.change_door_angle(state_angle)
            dd_mesh = cabinet_model_init.create_mesh()

            # T_O_D = np.linalg.inv(cabinet_model_init.T_D_A_init) @ cabinet_model_init.T_A_O
            T_O_D = np.linalg.inv(cabinet_model_init.T_D_A) @ np.linalg.inv(cabinet_model_init.T_A_O)

            # dd_mesh.transform(np.linalg.inv(cabinet_model_init.T_A_O))
            dd_mesh.transform(T_O_D)
            origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.zeros(3))
            plate_mesh = copy.deepcopy(cabinet_model_init.plate_mesh)
            plate_mesh.transform(T_O_D) 
            static_mesh = copy.deepcopy(cabinet_model_init.static_mesh)
            static_mesh.transform(T_O_D)
            o3d.visualization.draw_geometries([origin_rf, plate_mesh, static_mesh])

            visualize_poses(self.gripper_mesh, plate_mesh, self.T_G_DD_all_full[0:2000], self.T_TCP_G)
            
            collisions = collision_detection_fcl(self.gripper_mesh, self.T_G_DD_all_full, plate_mesh, T_TCP_G=self.T_TCP_G, static_mesh=static_mesh)
            collisions = np.array(collisions, dtype=bool)
            collisions = np.invert(collisions)

            self.T_G_DD_all_colfree = self.T_G_DD_all_full[collisions, ...]
            np.save(T_G_DD_all_colfree_path, self.T_G_DD_all_colfree)
            self.T_TCP_D_colfree = self.T_G_DD_all_colfree.reshape(-1, 4, 4) @ self.T_TCP_G[np.newaxis, ...]
        else:
            self.T_G_DD_all_colfree = np.load(T_G_DD_all_colfree_path)
            self.T_TCP_D_colfree = self.T_G_DD_all_colfree.reshape(-1, 4, 4) @ self.T_TCP_G[np.newaxis, ...]

        # Image capture parameters
        self.rgb_topic = '/camera/color/image_raw'
        self.depth_topic = '/camera/aligned_depth_to_color/image_raw'
        self.camera_info_topic = '/camera/color/camera_info'
        self.depth_encoding = '16UC1'
        
        self.home_q = None

        self.py_touch.create_simple_tool(a_tool, b_tool, c_tool, d_tool, h_tool, T_tool_6)
        camera_fu = 597.9033203125
        camera_fv = 598.47998046875
        camera_uc = 323.8436584472656
        camera_vc = 236.32774353027344
        camera_w = 640
        camera_h = 480
        self.py_touch.set_camera_params(camera_fu, camera_fv, camera_uc, camera_vc, camera_w, camera_h)
        self.touch_a = self.static_depth
        self.touch_b = 0.0
        self.touch_c = 0.005

        self.cabinet_filename = os.path.join(self.base_dir, f'cabinets_estimation.csv')
        self.cabinet_filename_gt = os.path.join(self.base_dir, f'cabinets_gt.csv')
        self.touches_filename = os.path.join(self.base_dir, f'cabinets_touches.csv')

        # FSM states
        self.state = "INITIALIZE"

    def initialize_for_scene(self, scene_idx, IS_OFFLINE=False):
        self.scene_idx = scene_idx
        rospy.loginfo(f"[FSM] Initializing for pose {self.scene_idx}...")

        self.base_detection_dir = os.path.join(self.base_dir, '{}_detection'.format('offline' if IS_OFFLINE else 'online'), 'cabinet_{}'.format(self.scene_idx))
        self.rvl_data_path = os.path.join(self.base_detection_dir, 'RVL_data')
        if not os.path.exists(self.rvl_data_path):
            os.makedirs(self.rvl_data_path)
        self.door_model_path = os.path.join(self.base_detection_dir, 'models/doorModel.json')
        self.cabinet_static_mesh_path = os.path.join(self.base_detection_dir, 'cabinet_model/cabinet_static.ply')
        self.cabinet_panel_mesh_path = os.path.join(self.base_detection_dir, 'cabinet_model/cabinet_panel.ply')
        self.ft_loss_T_6_0_path = os.path.join(self.base_detection_dir, 'ft_loss_T_6_0.npy')
        self.gt_door_model_path = os.path.join(self.base_detection_dir, 'models/T_A_W_gt.npy')
        self.gt_door_width_path = os.path.join(self.base_detection_dir, 'models/width.npy')
        self.best_hyp_path = os.path.join(self.base_detection_dir, 'DDT.txt')
        self.cabinet_mesh_path = os.path.join(self.base_detection_dir, 'cabinet_model/cabinet_mesh.ply')
        self.save_dir = os.path.join(self.base_detection_dir, 'detect_state_images')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)


    def initialize(self, is_new_scene=False):
        rospy.loginfo("[FSM] Initializing...")

        if is_new_scene:
            self.create_detected_cabinet_model(self.door_model_path)
            self.create_gt_cabinet_model()
        
        if not self.touch_session_set:
            # Set DDManipulator parameters
            self.set_rvl_manipulator()

        # self.save_models_info_for_rvl(self.state_angle)


    def set_rvl_manipulator(self, load_fcl_meshes=True):
        self.rvl_manipulator.set_robot_pose(self.robot.T_0_W)
        self.rvl_manipulator.set_door_model_params(
            self.cabinet_model.sx,
            self.cabinet_model.sy,
            self.cabinet_model.sz,
            self.cabinet_model.rx,
            self.cabinet_model.ry,
            self.cabinet_model.axis_pos,
            self.cabinet_model.side,
            self.cabinet_model.moving_to_static_part_distance)
        self.rvl_manipulator.set_door_pose(self.cabinet_model.T_A_W)
        self.rvl_manipulator.set_environment_state(self.state_angle)
        if load_fcl_meshes:
            self.rvl_manipulator.load_cabinet_static_mesh_fcl(self.cabinet_static_mesh_path)
            self.rvl_manipulator.load_cabinet_panel_mesh_fcl(self.cabinet_panel_mesh_path)

    def scale_T_G_DD_all(self, height_scale=0.8):
        height_scale_array = np.invert(self.T_TCP_D_colfree[:, 1, 3] > self.cabinet_model.sz * height_scale)
        self.T_G_DD_all = self.T_G_DD_all_colfree[height_scale_array, ...]

    def plan_trajectory(self, num_traj_pts=10, height_scale=0.8):
        # self.add_cabinet_model_to_scene(self.state_angle, is_doorless=True)
        self.remove_cabinet_model_from_scene() # The cabinet added in the generate_trajectories_and_approach3 function
        self.cabinet_model.mesh_save_path = self.cabinet_mesh_path
        rospy.loginfo("[FSM] Planning trajectory...")
        # T_G_DD_all = generate_tool_line_poses(self.cabinet_model.sz, self.T_TCP_G, self.R_TCP_D).reshape(-1, 4, 4)
        # self.scale_T_G_DD_all(height_scale=height_scale)

        T_TCP_D = self.T_TCP_D_colfree[np.invert(self.T_TCP_D_colfree[:, 1, 3] > self.cabinet_model.sz * height_scale)]

        self.waypoint_trajectories, self.trajectories = generate_trajectories_and_approach3(
            self.T_G_DD_all, 
            num_traj_pts, 
            self.state_angle, 
            self.opening_angle, 
            self.cabinet_model, 
            self.rvl_manipulator, 
            self.T_0_W, 
            self.robot,
            verbose=True,
            visualize=False,
            T_D_S=self.T_D_S,
            T_TCP_6=self.T_TCP_6
        )
        # np.save(os.path.join(self.base_detection_dir, 'trajectories.npy'), self.trajectories)

        states = np.linspace(self.state_angle, self.opening_angle, num_traj_pts)
        states = np.deg2rad(states)
        T_Arot_A_arr = np.zeros((num_traj_pts, 4, 4))
        T_Arot_A_arr[:, 2, 2] = 1.0
        T_Arot_A_arr[:, :3, :3] = rotz_multiple(states)
        T_A_W = self.cabinet_model.T_A_W
        T_Arot_W = T_A_W[np.newaxis, ...] @ T_Arot_A_arr
        self.T_D_S_array = T_Arot_W @ self.cabinet_model.T_D_Arot

        states_gt = np.linspace(self.state_angle_gt, self.opening_angle, num_traj_pts)
        states_gt = np.deg2rad(states_gt)
        T_Arot_A_arr = np.zeros((num_traj_pts, 4, 4))
        T_Arot_A_arr[:, 2, 2] = 1.0
        T_Arot_A_arr[:, :3, :3] = rotz_multiple(states_gt)
        T_A_W_gt = self.cabinet_gt.T_A_W
        T_Arot_W_gt = T_A_W_gt[np.newaxis, ...] @ T_Arot_A_arr
        self.T_D_S_array_gt = T_Arot_W_gt @ self.cabinet_gt.T_D_Arot


        return self.waypoint_trajectories, self.trajectories


    def load_trajectory(self):
        rospy.loginfo("[FSM] Loading trajectory...")
        trajectory_path = os.path.join(self.base_detection_dir, 'trajectories.npy')
        if os.path.exists(trajectory_path):
            self.trajectories = np.load(trajectory_path)

            if len(self.trajectories) < 1:
                rospy.logerr("[FSM] No trajectories loaded. Exiting.")
                self.state = "EXIT"
                return
            else:
                self.extract_closest_trajectory()
                # Save closest index to a file
                closest_idx_path = os.path.join(self.base_detection_dir, 'closest_idx.npy')
                np.save(closest_idx_path, self.closest_idx)

            rospy.loginfo("[FSM] Trajectory loaded successfully.")
            self.state = "EXECUTE_APPROACH_PATH"
        else:
            rospy.logerr("[FSM] Trajectory file not found. Exiting.")
            self.state = "EXIT"

    def extract_closest_trajectory(self, current_joints, trajectories, idx=0):
        rospy.loginfo("[FSM] Extracting closest trajectory...")
        first_joints = trajectories[:, idx, :]
        cheb_distances = np.max(np.abs(first_joints - current_joints), axis=1)
        self.closest_idx = np.argmin(cheb_distances)
        #debug
        # self.closest_idx = 9
        rospy.loginfo(f"[FSM] Closest trajectory index: {self.closest_idx}")
        return trajectories[self.closest_idx], self.closest_idx

    def zero_sensor(self):
        rospy.loginfo("[FSM] Zeroing force sensor...")
        self.robot.zero_ft_sensor()
        rospy.loginfo("[FSM] Force sensor zeroed.")

    def add_cabinet_model_to_scene(self, state_angle, is_doorless=False, scale_factor=1.0):
        rospy.loginfo("[FSM] Adding cabinet model to scene...")
        # self.cabinet_model.save_door_panel_mesh(self.cabinet_panel_mesh_path)
        self.robot.remove_mesh_from_scene("cabinet_model")

        TArot_A = np.eye(4)
        TArot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
        T_Arot_W = self.cabinet_model.T_A_W @ TArot_A

        mesh = copy.deepcopy(self.cabinet_model.static_mesh)
        center = np.array([0.0, 0.0, 0.0])

        # mesh.transform(np.linalg.inv(self.cabinet_model.T_A_O))
        self.cabinet_model.change_door_angle(state_angle)
        self.cabinet_model.create_mesh()
        T_O_D = np.linalg.inv(self.cabinet_model.T_D_A) @ np.linalg.inv(self.cabinet_model.T_A_O)

        if is_doorless:
            # Add doorless cabinet model to the scene
            # static_mesh.transform(np.linalg.inv(self.cabinet_model.T_A_O_init))
            # center = np.array([0.0, 0.0, 0.0])
            mesh.scale(scale_factor, center)
            mesh_path = self.cabinet_static_mesh_path
            self.cabinet_model.save_mesh_without_doors(mesh_path, mesh=mesh, pose=T_O_D)
        else:
            mesh = copy.deepcopy(self.cabinet_model.mesh)
            
            mesh.scale(scale_factor, center)
            mesh_path = self.cabinet_mesh_path
            self.cabinet_model.save_mesh(mesh_path, mesh=mesh, pose=T_O_D)

        self.robot.add_mesh_to_scene(mesh_path, "cabinet_model", self.T_D_S)

        # # Add drawer part box above the cabinet
        # drawer_z = 0.35 * scale_factor
        # drawer_x = self.cabinet_model.static_d * scale_factor
        # drawer_y = (self.cabinet_model.sy + 2 * self.cabinet_model.side + 3 * self.cabinet_model.moving_to_static_part_distance) * scale_factor
        # self.robot.remove_box_from_scene("drawer_part_box")
        # T_O_S = T_Arot_W @ np.linalg.inv(self.cabinet_model.T_A_O)
        # T_center_S = T_O_S.copy()
        # T_center_S[:3, 3] = T_O_S[:3, :3] @ center + T_O_S[:3, 3]
        # T_center_S[:3, 3] += T_center_S[:3, 2] * ((self.cabinet_model.sz * 0.5 + self.cabinet_model.moving_to_static_part_distance + self.cabinet_model.side) * scale_factor + drawer_z * 0.5)
        # self.robot.add_box_to_scene(
        #     "drawer_part_box",
        #     T_center_S,
        #     np.array([drawer_x, drawer_y, drawer_z]))

        rospy.loginfo("[FSM] Cabinet model added to scene.")

    def remove_cabinet_model_from_scene(self):
        rospy.loginfo("[FSM] Removing cabinet model from scene...")
        self.robot.remove_mesh_from_scene("cabinet_model")
        self.robot.remove_box_from_scene("drawer_part_box")
        rospy.loginfo("[FSM] Cabinet model removed from scene.")

    def execute_insertion_path(self, insertion_trajectory: np.ndarray, force_threshold=10.0, state_angle=0.0):
        # Zero the sensor
        rospy.loginfo("[FSM] Zeroing the sensor...")
        self.zero_sensor()

        rospy.loginfo("[FSM] Executing insertion path...")

        success = self.execute_with_monitoring(insertion_trajectory, force_threshold=force_threshold, max_velocity=0.05, max_acceleration=0.05)

        skipped_touch = False

        if not success:
            user_input = input('[FSM] Insertion path execution failed. Press 1 to record the touch or 0 to skip: ')
            if user_input == '0':
                # Skip recording
                skipped_touch = True

            rospy.logwarn("[FSM] Insertion path execution failed. Backing up.")
            # backup trajectory
            current_joints = self.robot.get_current_joint_values()
            backup_trajectory = np.array([current_joints, insertion_trajectory[0]])
            _, T_6_0_ft_loss = self.execute_and_remember_joints_on_force_loss(backup_trajectory, drop_delta_treshold=force_threshold-1.0, low_force_threshold=1.0)
            if T_6_0_ft_loss is not None:
                T_6before_0 = self.robot.get_fwd_kinematics_moveit(insertion_trajectory[-2])
                T_6insert_0 = self.robot.get_fwd_kinematics_moveit(insertion_trajectory[-1])
                self.set_touch(T_6_0_ft_loss, 
                            np.array((T_6before_0, T_6insert_0)), 
                            touch_type=TouchType.UNWANTED_TOUCH,
                            state_angle=state_angle)
                if self.CORRECT_ON_TOUCH:
                    self.correct_touch_model()

        return success, skipped_touch


    def correct_touch_model(self):
        rospy.loginfo("[FSM] Correcting touch model...")

        # Correct the model
        self.py_touch.correct()

        self.update_model_from_touch()

    def update_model_from_touch(self):
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(self.state_angle))

        # Update model x to py_touch
        self.rvl_manipulator.update_model_x()

        T_Arot_6_corrected = self.rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
        T_C_6_corrected = self.rvl_manipulator.get_corrected_camera_pose().astype(np.float64)

        if np.any(np.isnan(T_Arot_6_corrected)) or np.any(np.isnan(T_C_6_corrected)):
            rospy.logerr("[FSM] Corrected poses contain NaN values. Sticking to the previous model.")
            return
        
        # Update the poses
        self.T_C_6 = T_C_6_corrected.copy()
        self.T_A_C = np.linalg.inv(self.T_C_6) @ T_Arot_6_corrected @ np.linalg.inv(T_Arot_A)

        T_D_Arot = self.rvl_manipulator.get_corrected_pose_D_Arot().astype(np.float64)
        # self.cabinet_model.T_D_Arot[:3, 3] = T_D_Arot[:3, 3].copy()
        self.cabinet_model.T_D_Arot = T_D_Arot.copy()

        # Update the cabinet model
        self.cabinet_model.T_A_W = self.robot.T_0_W @ self.T_6_0_capture @ self.T_C_6 @ self.T_A_C
        print(f"[FSM] Corrected cabinet model pose: {self.cabinet_model.T_A_W}")

        # self.T_D_S = self.cabinet_model.T_A_W @ T_Arot_A @ self.cabinet_model.T_D_Arot
        T_D_0 = self.rvl_manipulator.get_corrected_pose_D_0().astype(np.float64)

        self.T_D_S = self.robot.T_0_W @ T_D_0

        self.rvl_manipulator.set_pose_DD_S(self.T_D_S)

        self.rvl_manipulator.set_environment_from_touch()

        # # visualize
        # cabinet_mesh_save_path = os.path.join(self.base_dir, 'cabinet_model/cabinet_mesh_corrected.ply')
        # self.cabinet_model.save_mesh(cabinet_mesh_save_path)
        # # o3d
        # mesh = o3d.io.read_triangle_mesh(cabinet_mesh_save_path)
        # mesh.transform(self.cabinet_model.T_A_W)
        # DD_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.zeros(3))
        # DD_rf.transform(self.T_D_S)
        # A_S_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.zeros(3))
        # A_S_rf.transform(self.cabinet_model.T_A_W)
        # o3d.visualization.draw_geometries([mesh, DD_rf, A_S_rf])

    def set_touch(self, T_6contact_0, T_6via_0_pts, touch_type: TouchType, state_angle: float, only_save=False):
        """Set touch in RVL and save the touch information for RVL.
        Args:
            T_6contact_0 (np.ndarray): The contact pose in the robot frame.
            T_6via_0 (np.ndarray): The via pose in the robot frame.
            V (np.ndarray): The unit vector of the approach path.
            touch_type (TouchType): The type of touch.
        """
        T_6contact_6 = np.linalg.inv(self.T_6_0_capture) @ T_6contact_0
        T_6via_6 = np.linalg.inv(self.T_6_0_capture)[np.newaxis, ...] @ T_6via_0_pts
        V = T_6via_6[1, :3, 3] - T_6via_6[0, :3, 3]
        t = np.linalg.norm(V)
        V /= t  # Normalize the vector
        self.save_touches_info_for_rvl(T_6contact_6, V, t, touch_type, state_angle)
        b_miss = True if touch_type == TouchType.MISS else False
        if not only_save:
            self.py_touch.set_touch(T_6contact_6, V, t, b_miss)


    def recapture(self, trajectory):
        rospy.loginfo("[FSM] Replanning...")

        # Get estimated door state
        # Find the closest point in the trajectory

        estimated_angle = self.state_angle
        print(f"[replan] Estimated door angle at contact loss: {estimated_angle:.2f} degrees")

        # Get new camera capture pose
        T_C_W_new, joints_camera_capture = get_camera_pose_on_sphere_distance(self.cabinet_model, 
                                                                              self.robot, 
                                                                              self.T_6_0_capture,
                                                                              estimated_angle)
        if joints_camera_capture is None:
            joints_camera_capture = self.robot.get_closest_ik_solution(self.T_6_0_capture, None)
            
            if joints_camera_capture is None:
                rospy.logerr("[FSM] No valid camera capture pose found. Exiting.")
                self.state = "EXIT"
                return
            
            T_C_W_new = self.robot.T_0_W @ self.T_6_0_capture @ self.robot.T_C_6

        # Backup from the current state
        T_6_0_current = self.robot.get_current_tool_pose()
        T_6_0_backup = T_6_0_current.copy()
        T_6_0_backup[:3, 3] = T_6_0_backup[:3, 3] - T_6_0_backup[:3, 2] * 0.05 # move 5 cm in z-direction
        joints_backup = self.robot.get_closest_ik_solution(T_6_0_backup, None) # get ik solution for backup and take current joints
        if joints_backup is None:
            rospy.logerr("[FSM] No valid IK solution for backup. Exiting.")
            input("Move the robot by hand to a valid position and press Enter to continue...")
        else:
            success = False
            while not success:
                backup_trajectory = np.array([self.robot.get_current_joint_values(), joints_backup])
                success = self.execute_with_monitoring(backup_trajectory, force_threshold=30.0, max_velocity=0.1, max_acceleration=0.1)

        # # Add cabinet model to the robot environment in Moveit
        self.cabinet_model.change_door_angle(estimated_angle)
        # self.cabinet_model.save_mesh(self.cabinet_mesh_path)
        # self.robot.remove_mesh_from_scene("cabinet_model")
        # self.robot.add_mesh_to_scene(self.cabinet_mesh_path, "cabinet_model", self.cabinet_model.T_A_W)
        self.add_cabinet_model_to_scene(estimated_angle, scale_factor=1.05)

        # Move to the new camera capture pose
        success = False
        while not success:
            trajectory_capture = np.array([self.robot.get_current_joint_values(), joints_camera_capture])
            planned_joints, _ = self.robot.plan_to_joint_goals2(trajectory_capture)

        success = self.execute_with_monitoring(planned_joints, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.3)

        # Remove cabinet model from the robot environment
        self.robot.remove_mesh_from_scene("cabinet_model")

        # Capture door state
        image_capture = OneShotImageCapture(self.save_dir, self.rgb_topic, self.depth_topic, self.camera_info_topic, self.depth_encoding)
        rgb_path, _, ply_path = image_capture.capture_single_image_and_save()
        T_C_W_init_capture = self.robot.T_0_W @ self.T_6_0_capture @ self.T_C_6_init
        # T_Cdet_Cdiff = np.linalg.inv(T_C_W_init_capture) @ T_C_W_new
        T_Cdiff_Cdet = np.linalg.inv(T_C_W_new) @ T_C_W_init_capture
        door_state_detector = DoorStateDetector(detector_config_path=self.door_detector_config_path, best_hyp_path=self.best_hyp_path)
        self.state_angle = door_state_detector.detect_state(rgb_path, ply_path, T_Cdiff_Cdet)
        self.state_angle = self.state_angle[0] # take the first element
        print(f"[replan] Detected door state: {self.state_angle}")

        # Set new touch scene
        self.set_new_touch_scene(self.state_angle, self.state_angle)
        self.update_model_from_touch()

        # Plan new trajectory
        self.state = "PLAN_TRAJECTORY"


    def save_models_init_info_for_rvl(self):
        if not os.path.exists(self.cabinet_filename):
            with open(self.cabinet_filename, 'w') as f:
                header = 'session_idx,scene_idx,load_idx,sx,sy,sz,rx,ry,state_angle,R_A_C,t_A_C,R_C_E,t_C_E,R_E_0,t_E_0'
                f.write(header + '\n')
        else:
            # Get last line and check the session_idx
            with open(self.cabinet_filename, 'r') as f:
                lines = f.readlines()
                if len(lines) > 1:
                    last_line = lines[-1].strip()
                    self.session_idx = int(last_line.split(',')[0]) + 1
                else:
                    self.session_idx = 0
        if not os.path.exists(self.cabinet_filename_gt):
            with open(self.cabinet_filename_gt, 'w') as f:
                header = 'session_idx,scene_idx,sx,sy,sz,rx,ry,state_angle,R_A_S,t_A_S'
                f.write(header + '\n')
        if not os.path.exists(self.touches_filename):
            with open(self.touches_filename, 'w') as f:
                header = 'session_idx,scene_idx,type,R_Ek_E,t_Ek_E,V,t,state_angle'
                f.write(header + '\n')


    def save_models_info_for_rvl(self, state_angle=0):
        # For detected cabinet model, index, dimensions, self.T_A_C, T_C_E and T_E_0_capture are saved
        T_Arot_C = np.eye(4)
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))

        T_Arot_C = self.T_A_C @ T_Arot_A
        with open(self.cabinet_filename, 'a') as f:
            txt = f'{self.session_idx},{self.idx_to_save},{self.scene_idx},{self.cabinet_model.sx},{self.cabinet_model.sy},{self.cabinet_model.sz},{self.cabinet_model.rx},{self.cabinet_model.ry},{state_angle}'
            R_A_C = self.T_A_C_init[:3,:3].reshape(-1)
            t_A_C = self.T_A_C_init[:3, 3].reshape(-1)
            for i in range(9):
                txt += f',{R_A_C[i]}'
            for i in range(3):
                txt += f',{t_A_C[i]}'
            R_C_E = self.T_C_6_init[:3,:3].reshape(-1)
            t_C_E = self.T_C_6_init[:3, 3].reshape(-1)
            for i in range(9):
                txt += f',{R_C_E[i]}'
            for i in range(3):
                txt += f',{t_C_E[i]}'
            R_E_0_capture = self.T_6_0_capture[:3,:3].reshape(-1)
            t_E_0_capture = self.T_6_0_capture[:3, 3].reshape(-1)
            for i in range(9):
                txt += f',{R_E_0_capture[i]}'
            for i in range(3):
                txt += f',{t_E_0_capture[i]}'
            f.write(txt + '\n')
        
        T_Arotgt_A = np.eye(4)
        T_Arotgt_A[:3, :3] = rot_z(np.deg2rad(self.state_angle_gt))
        T_Arot_W = self.cabinet_gt.T_A_W @ T_Arot_A

        with open(self.cabinet_filename_gt, 'a') as f:
            R_A_S_gt = T_Arot_W[:3,:3].reshape(-1)
            t_A_S_gt = T_Arot_W[:3, 3].reshape(-1)
            txt = f'{self.session_idx},{self.idx_to_save},{self.cabinet_gt.sx},{self.cabinet_gt.sy},{self.cabinet_gt.sz},{self.cabinet_gt.rx},{self.cabinet_gt.ry},{self.state_angle_gt}'
            for i in range(9):
                txt += f',{R_A_S_gt[i]}'
            for i in range(3):
                txt += f',{t_A_S_gt[i]}'
            f.write(txt + '\n')


    def save_touches_info_for_rvl(self, T_Ek_E, V, t, touch_type: TouchType, state_angle):
        """Save touch information for RVL.
        Args:
            T_Ek_E (np.ndarray): The contact pose in the robot frame.
            V (np.ndarray): The unit vector of the approach path.
            touch_type (TouchType): The type of touch.
            state_angle (float): The state angle of the door.
        """
        R_Ek_E = T_Ek_E[:3, :3].reshape(-1)
        t_Ek_E = T_Ek_E[:3, 3].reshape(-1)
        V_flat = V.reshape(-1)
        with open(self.touches_filename, 'a') as f:
            txt = f'{self.session_idx},{self.idx_to_save},{touch_type.value}'
            for i in range(9):
                txt += f',{R_Ek_E[i]}'
            for i in range(3):
                txt += f',{t_Ek_E[i]}'
            for i in range(3):
                txt += f',{V_flat[i]}'
            txt += f',{t}'
            txt += f',{state_angle}'
            f.write(txt + '\n')
            

    def execute_with_monitoring(self, trajectory, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.5):
        """Execute the trajectory with force monitoring."""
        monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(self.robot, force_threshold))
        monitor_thread.start()
        success = self.robot.send_joint_trajectory_action2(trajectory, max_velocity=max_velocity, max_acceleration=max_acceleration)
        monitor_thread.join()
        return success
    
    def execute_with_monitoring_remember_joints(self, trajectory, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.5):
        collision_joints = []
        monitor_thread = threading.Thread(target=monitor_force_and_cancel_remember_joints, args=(
            self.robot, collision_joints, force_threshold, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        success = self.robot.send_joint_trajectory_action2(trajectory, max_velocity=max_velocity, max_acceleration=max_acceleration)
        monitor_thread.join()
        return success, collision_joints

    def execute_without_monitoring(self, trajectory):
        success = self.robot.send_joint_trajectory_action2(trajectory, max_velocity=0.5, max_acceleration=0.5)
        return success

    def execute_and_remember_joints_on_force_loss(self, trajectory, drop_delta_treshold=2.0, low_force_threshold=1.5):
        ft_loss_joints = [] # reset the joints
        T_6_0_ft_loss = None
        monitor_thread = threading.Thread(target=monitor_force_drop_and_remember_joints, args=(self.robot, ft_loss_joints, drop_delta_treshold, low_force_threshold, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        self.robot.send_joint_trajectory_action2(trajectory, max_velocity=0.05, max_acceleration=0.05)
        monitor_thread.join()

        ft_loss_joints = np.array(ft_loss_joints)
        if ft_loss_joints.shape[0] == 1:
            rospy.logwarn("[FSM] Force loss detected during backup. Remembered joints: %s", ft_loss_joints[0])
            T_6_0_ft_loss = self.robot.get_fwd_kinematics_moveit(ft_loss_joints[0].tolist())
        return ft_loss_joints, T_6_0_ft_loss

    def create_detected_cabinet_model(self, data_model_path):
        with open(data_model_path, 'r') as f:
            data = json.load(f)

        R = np.array(data["R"])
        t = np.array(data["t"])
        self.s = np.array(data["s"])
        # self.r = np.array(data["r"])
        self.r = np.array([-self.door_thickness * 0.5, -self.s[0] * 0.5])  # static side position
        axis_pos = data["openingDirection"]
        self.T_A_C = np.eye(4)
        self.T_A_C[:3, :3] = R
        self.T_A_C[:3, 3] = t
        # self.T_A_C[:3, 3] *= self.scale_factor

        self.T_A_C_init = self.T_A_C.copy()

        state_angle = axis_pos * np.rad2deg(np.arcsin(self.push_latch_mechanism_length / (self.s[0] - self.latch_offset)))
        joint_values = np.array(data["joint_values"])

        T_6_0 = self.robot.get_fwd_kinematics_moveit(joint_values)
        np.save(os.path.join(self.rvl_data_path, 'T_6_0_capture.npy'), T_6_0)
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
        T_A_6 = self.T_C_6 @ self.T_A_C @ T_Arot_A
        np.save(os.path.join(self.rvl_data_path, 'T_A_6.npy'), T_A_6)
        T_A_W = self.T_0_W @ T_6_0 @ self.T_C_6 @ self.T_A_C
        self.cabinet_model = Cabinet2(s=np.array([self.door_thickness, self.s[0], self.s[1], self.static_depth]),
                                r=self.r,
                                axis_pos=axis_pos,
                                T_A_W=T_A_W,
                                save_path=None,
                                has_handle=False)

        self.T_6_0_capture = T_6_0
        self.joint_values_capture = joint_values
        self.state_angle = state_angle

    def create_gt_cabinet_model(self):
        width = self.gt_width
        height = self.gt_height
        axis_pos = -1
        T_A_W_gt = self.selected_poses[self.scene_idx][0].copy()

        self.state_angle_gt = axis_pos * np.rad2deg(np.arcsin(self.push_latch_mechanism_length / (width - self.latch_offset)))

        self.cabinet_gt = Cabinet2(s=np.array([self.door_thickness, width, height, self.static_depth]),
                                r=np.array([-self.door_thickness*0.5, -width*0.5]),
                                axis_pos=axis_pos,
                                T_A_W=T_A_W_gt,
                                save_path=None,
                                has_handle=False)
   

    def go_to_home_position(self):
        """Go to the home position of the robot."""
        if self.home_q is None:
            rospy.logerr("[FSM] Home position is not set. Cannot go to home position.")
            self.home_q = np.array([-np.pi/2, -np.pi/2, 0.0, -np.pi/2, 0.0, -3/4 * np.pi])

        # Check if the robot is in the home position
        current_joints = self.robot.get_current_joint_values()
        if np.allclose(current_joints, self.home_q, atol=0.01):
            rospy.loginfo("[FSM] Robot is already in the home position.")
            return
        
        self.add_cabinet_model_to_scene(self.state_angle, scale_factor=1.05)
        home_trajectory = np.vstack((current_joints, self.home_q))
        traj_moveit, success = self.robot.plan_to_joint_goals2(home_trajectory)
        traj = traj_moveit if success else home_trajectory

        for _ in range(2):
            success = self.execute_with_monitoring(traj, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.5)
            if not success:
                input("Failed to go to home position, move the robot to a position and try again. Press Enter to retry...")
                traj = np.vstack((self.robot.get_current_joint_values(), self.home_q))
            else:
                break

    def get_closest_contact_pose_idx(self, contact_poses, T_6_0):
        # Calculate the angle based on the distance from the contact points
        contact_positions = contact_poses[:, :3, 3]
        distances = np.linalg.norm(contact_positions - T_6_0[:3, 3], axis=1)
        closest_idx = np.argmin(distances)
        return closest_idx

    def set_new_touch_scene(self, state_angle, state_angle_gt):
        self.idx_to_save += 1
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))

        T_Arot_A_gt = np.eye(4)
        T_Arot_A_gt[:3, :3] = rot_z(np.deg2rad(state_angle_gt))
        T_Arot_0_gt = self.cabinet_gt.T_A_W @ T_Arot_A_gt

        self.py_touch.set_new_scene(self.door_thickness, self.s[0], self.s[1], self.r[0], self.r[1], self.touch_a, self.touch_b, self.touch_c, state_angle, self.T_C_6_init, self.T_A_C_init, self.T_6_0_capture,
                                    self.cabinet_gt.sx, self.cabinet_gt.sy, self.cabinet_gt.sz, self.cabinet_gt.rx, self.cabinet_gt.ry, state_angle_gt, T_Arot_0_gt)
        self.save_models_info_for_rvl(state_angle)


    def load_session(self):
        def get_matrix(base, size, row_, col_idx_):
            start = col_idx_[base]
            return np.array(row_[start:start + size], dtype=np.float64)

        def compute_column_indices(header):
            col_idx = {}
            idx = 0
            # while idx < len(header):
            for name in header:
                # name = header[idx]
                if name in {"R_A_C", "R_C_E", "R_E_0", 'R_A_S', 'R_Ek_E'}:
                    col_idx[name] = idx
                    idx += 9
                elif name in {"t_A_C", "t_C_E", "t_E_0", 't_A_S', 't_Ek_E', 'V'}:
                    col_idx[name] = idx
                    idx += 3
                else:
                    col_idx[name] = idx
                    idx += 1
            return col_idx

        idx_to_save = -1
        type_touch = -1
        succ_trajs_all = 0
        is_really_offline = False
        succ_trajs = 0

        with open(self.cabinet_filename, 'r') as f:
            reader = csv.reader(f)

            header = next(reader)

            # col_idx = {col: idx for idx, col in enumerate(header)}
            col_idx = compute_column_indices(header)

            rows = list(reader)
            if not rows:
                self.session_idx = 0
                raise RuntimeError("No session found, please run the script to create a session.")
        
            last_row = rows[-1]
            self.session_idx = int(last_row[col_idx['session_idx']])  # Load last session
            prev_load_idx = -1
            last_scene_idx = int(last_row[col_idx['scene_idx']])

            for row in rows:
                sess_idx = int(row[col_idx['session_idx']])
                if sess_idx != self.session_idx: # load only last session
                    continue

                scene_idx = int(row[col_idx['scene_idx']])
                load_idx = int(row[col_idx['load_idx']])

                if prev_load_idx != load_idx: 
                    succ_trajs = 0

                # is_really_offline = True if load_idx < 3 and self.IS_OFFLINE else False
                is_really_offline = True if succ_trajs_all < 9 and self.IS_OFFLINE else False

                sx = float(row[col_idx['sx']])
                sy = float(row[col_idx['sy']])
                sz = float(row[col_idx['sz']])

                self.s = np.array([sy, sz])

                rx = float(row[col_idx['rx']])
                ry = float(row[col_idx['ry']])

                self.r = np.array([rx, ry])

                state_angle = float(row[col_idx['state_angle']])
                self.state_angle = state_angle

                R_A_C = get_matrix('R_A_C', 9, row, col_idx).reshape(3, 3)
                t_A_C = get_matrix('t_A_C', 3, row, col_idx)
                R_C_E = get_matrix('R_C_E', 9, row, col_idx).reshape(3, 3)
                t_C_E = get_matrix('t_C_E', 3, row, col_idx)
                R_E_0_capture = get_matrix('R_E_0', 9, row, col_idx).reshape(3, 3)
                t_E_0_capture = get_matrix('t_E_0', 3, row, col_idx)

                self.T_A_C = np.eye(4)
                self.T_A_C[:3, :3] = R_A_C
                self.T_A_C[:3, 3] = t_A_C

                self.T_A_C_init = self.T_A_C.copy()

                T_C_E = np.eye(4)
                T_C_E[:3, :3] = R_C_E
                T_C_E[:3, 3] = t_C_E
                # self.T_C_6 = T_C_E.copy()

                T_E_0_capture = np.eye(4)
                T_E_0_capture[:3, :3] = R_E_0_capture
                T_E_0_capture[:3, 3] = t_E_0_capture

                self.T_6_0_capture = T_E_0_capture.copy()

                T_A_W = self.robot.T_0_W @ T_E_0_capture @ T_C_E @ self.T_A_C
                self.cabinet_model = Cabinet2(s=np.array([sx, sy, sz, self.static_depth]),
                                        r=np.array([rx, ry]),
                                        axis_pos=-1,
                                        T_A_W=T_A_W,
                                        save_path=None,
                                        has_handle=False)
                
                # Load GT cabinet model
                with open(self.cabinet_filename_gt, 'r') as f_gt:

                    reader_gt = csv.reader(f_gt)

                    header_gt = next(reader_gt)
                    col_idx_gt = compute_column_indices(header_gt)

                    rows_gt = list(reader_gt)

                    for row_gt in rows_gt:

                        gt_sess_idx = int(row_gt[col_idx_gt['session_idx']])
                        if gt_sess_idx != sess_idx:
                            continue
                        gt_scene_idx = int(row_gt[col_idx_gt['scene_idx']])
                        if gt_scene_idx != scene_idx:
                            continue

                        sx_gt = float(row_gt[col_idx_gt['sx']])
                        sy_gt = float(row_gt[col_idx_gt['sy']])
                        sz_gt = float(row_gt[col_idx_gt['sz']])
                        rx_gt = float(row_gt[col_idx_gt['rx']])
                        ry_gt = float(row_gt[col_idx_gt['ry']])
                        self.state_angle_gt = float(row_gt[col_idx_gt['state_angle']])

                        R_Arot_W_gt = get_matrix('R_A_S', 9, row_gt, col_idx_gt).reshape(3, 3)
                        t_Arot_W_gt = get_matrix('t_A_S', 3, row_gt, col_idx_gt)

                        T_Arot_W_gt = np.eye(4)
                        T_Arot_W_gt[:3, :3] = R_Arot_W_gt
                        T_Arot_W_gt[:3, 3] = t_Arot_W_gt

                        T_Arot_A_gt = np.eye(4)
                        T_Arot_A_gt[:3, :3] = rot_z(np.deg2rad(self.state_angle_gt))
                        T_A_W_gt = T_Arot_W_gt @ np.linalg.inv(T_Arot_A_gt)

                        self.cabinet_gt = Cabinet2(s=np.array([sx_gt, sy_gt, sz_gt, self.static_depth]),
                                                r=np.array([rx_gt, ry_gt]),
                                                axis_pos=-1,
                                                T_A_W=T_A_W_gt,
                                                save_path=None,
                                                has_handle=False)

                if not self.touch_session_set:
                    self.set_rvl_manipulator(load_fcl_meshes=False)


                # if scene_idx == last_scene_idx:
                #     self.py_touch.set_visualization(True)
                #     debug_ = 0
                #     print('debug')

                self.py_touch.set_new_scene(sx, sy, sz, rx, ry, self.touch_a, self.touch_b, self.touch_c, self.state_angle, T_C_E, self.T_A_C_init, T_E_0_capture,
                                            sx_gt, sy_gt, sz_gt, rx_gt, ry_gt, self.state_angle_gt, T_Arot_W_gt)
                self.touch_session_set = True
                self.update_model_from_touch()
                # self.correct_touch_model()
                # plan trajectory from estimated cabinet
                self.initialize_for_scene(load_idx, is_really_offline)


                # self.rvl_manipulator.set_environment_from_touch()
                # self.cabinet_model.create_mesh()
                # self.cabinet_model.save_mesh('/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/offline_detection/cabinet_1/cabinet_model/cabinet_mesh.ply', 
                #                              mesh=self.cabinet_model.mesh, pose=np.linalg.inv(self.cabinet_model.T_A_O))
                # self.rvl_manipulator.visualize_vn_model()


                if scene_idx == last_scene_idx:
                    num_traj_pts = 3 if is_really_offline else 10
                    self.opening_angle = self.state_angle - 5.0 if is_really_offline else -45.0
                    # self.scale_T_G_DD_all()
                    # waypoint_trajectories, trajectories = self.plan_trajectory(num_traj_pts)
                    waypoint_trajectories, trajectories = [], []

                with open(self.touches_filename, 'r') as f_touches:
                    reader_touches = csv.reader(f_touches)
                    header_touches = next(reader_touches)
                    col_idx_touches = compute_column_indices(header_touches)

                    rows_touches = list(reader_touches)
                    if not rows_touches:
                        continue

                    for row_touch in rows_touches:
                        touch_sess_idx = int(row_touch[col_idx_touches['session_idx']])
                        if touch_sess_idx != sess_idx:
                            continue
                        touch_scene_idx = int(row_touch[col_idx_touches['scene_idx']])
                        if touch_scene_idx != scene_idx:
                            continue
                        
                        type_touch = TouchType(int(row_touch[col_idx_touches['type']]))
                        if type_touch == TouchType.WANTED_TOUCH:
                            succ_trajs += 1
                            succ_trajs_all += 1
                            continue  # Skip wanted touches for now

                        R_Ek_E = get_matrix('R_Ek_E', 9, row_touch, col_idx_touches).reshape(3, 3)
                        t_Ek_E = get_matrix('t_Ek_E', 3, row_touch, col_idx_touches)
                        V = get_matrix('V', 3, row_touch, col_idx_touches)
                        t = float(row_touch[col_idx_touches['t']])
                        state_angle_touch = float(row_touch[col_idx_touches['state_angle']])
                        b_miss = True if type_touch == TouchType.MISS else False

                        T_Ek_E = np.eye(4)
                        T_Ek_E[:3, :3] = R_Ek_E
                        T_Ek_E[:3, 3] = t_Ek_E

                        self.py_touch.set_touch(T_Ek_E, V, t, b_miss)

                        if self.CORRECT_ON_TOUCH:
                            if scene_idx == 2:
                                print('debug')
                            self.correct_touch_model()
                            # if scene_idx == last_scene_idx and load_idx == prev_load_idx:
                                # self.rvl_manipulator.visualize_vn_current_state(np.zeros((6,)), np.eye(4))
                
                prev_load_idx = load_idx


        # self.py_touch.set_visualization(True)
        # if self.CORRECT_ON_TOUCH:
        #     self.correct_touch_model()

        # self.initialize_for_scene(load_idx, is_really_offline)
        # if load_idx == prev_load_idx:
        #     num_traj_pts = 3 if is_really_offline else 10
        #     self.opening_angle = self.state_angle - 5.0 if is_really_offline else -45.0
        #     self.scale_T_G_DD_all()
        #     waypoint_trajectories, trajectories = self.plan_trajectory(num_traj_pts)


        # scenes_done, trajs_done, self.idx_to_save, IS_OFFLINE_, is_plan_traj
        is_plan_traj = False
        trajs_done = 0
        # IS_OFFLINE_ = self.IS_OFFLINE
        IS_OFFLINE_ = is_really_offline
        is_new_scene = False
        if IS_OFFLINE_:
            if load_idx <= 2:
                if succ_trajs > 2:
                    load_idx += 1
                    is_new_scene = True
                    succ_trajs = 0
                else:
                    is_plan_traj = True

            if load_idx > 2:
                IS_OFFLINE_ = False
                load_idx = succ_trajs_all - 9
            trajs_done = succ_trajs

        if not IS_OFFLINE_:
            is_plan_traj = True
            if succ_trajs > 0:
                load_idx += 1
                trajs_done = 0
                is_new_scene = True

        return load_idx, trajs_done, scene_idx, IS_OFFLINE_, is_plan_traj, is_new_scene, waypoint_trajectories, trajectories

    def get_all_contact_positions_A(self, pos_tol, k) -> List[np.ndarray]:
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(self.state_angle))
        T_TCP_Arot = self.cabinet_model.T_D_Arot[np.newaxis, ...] @ self.T_G_DD_all @ self.T_TCP_G[np.newaxis, ...]
        contact_positions_ = T_TCP_Arot[:, :3, 3]  # Extract the contact positions
        
        if k == 1:
            # If k is 1, return a random position
            indices = np.random.choice(len(contact_positions_), size=1, replace=False)
            return np.array(contact_positions_[indices])
        
        indices = self.pick_distributed_groups(contact_positions_, pos_tol, k)
        contact_positions = [contact_positions_[i] for i in indices]
        contact_positions = np.array(contact_positions)
        np.random.shuffle(contact_positions) # Shuffle the contact positions
        return contact_positions
    
    def get_contact_positions_A(self, waypoint_trajectories: List[Waypoint]) -> np.ndarray:
        contact_positions = []
        # Select contact positions from the trajectories
        for trajectory_wp in waypoint_trajectories:
            trajectory = trajectory_wp.get_full_trajectory_task_space()
            T_6_0_contact = trajectory[2]

            T_Arot_A = np.eye(4)
            T_Arot_A[:3, :3] = rot_z(np.deg2rad(self.state_angle))
            T_Arot_W = self.cabinet_model.T_A_W @ T_Arot_A
            T_6_Arot = np.linalg.inv(T_Arot_W) @ np.linalg.inv(self.robot.T_0_W) @ T_6_0_contact
            T_TCP_Arot = T_6_Arot @ self.robot.T_G_6 @ self.T_TCP_G
            contact_positions.append(T_TCP_Arot[:3, 3])
        contact_positions = np.array(contact_positions)
        return contact_positions

    def gaussian_weighted_choice(self, candidates, distances, sigma=0.01, normalize=False):
        if normalize:
            max_dist = distances.max()
            if max_dist > 0:
                distances = distances / max_dist  # Scale to [0, 1]

        weights = np.exp(-(distances ** 2) / (2 * sigma ** 2))
        weights /= np.sum(weights)
        
        return np.random.choice(candidates, p=weights)

    def gaussian_weighted_choice_fixed_scale(self,
        candidates,
        contact_z_ref,
        candidate_positions_z,
        door_height=1.0,
        sigma_ratio=0.1
    ):
        sigma = sigma_ratio * door_height
        sampled_z = np.random.normal(loc=contact_z_ref, scale=sigma)
        # Find the candidate closest to the sampled Z
        idx = np.argmin(np.abs(candidate_positions_z - sampled_z))
        return candidates[idx]

    def visualize_gaussian_probabilities(self, points: np.ndarray, ref_index: int, sigma: float = 0.02):
        assert points.ndim == 2 and points.shape[1] == 3, "Points must be of shape (N, 3)."
        assert 0 <= ref_index < points.shape[0], "ref_index must be within [0, N-1]."

        # Compute Gaussian probabilities
        ref_point = points[ref_index]
        dists = np.linalg.norm(points - ref_point, axis=1)
        weights = np.exp(-(dists**2) / (2 * sigma**2))
        probs = weights / np.sum(weights)

        # Visualization
        plt.figure(figsize=(8, 4))
        plt.scatter(dists, [0]*len(points), c=probs, s=probs * 1000, cmap='viridis')
        plt.colorbar(label="Selection Probability")
        plt.xlabel("Door Line (m)")
        plt.title(f"Gaussian Probability Distribution (σ={sigma:.3f} m)")
        plt.axvline(0., color='r', linestyle='--', label='Reference Point')
        plt.legend()
        plt.show()

        return probs

    def pick_z_distributed_indices(self, contact_positions: np.ndarray, k: int) -> List[int]:
        assert contact_positions.shape[0] >= k, "Not enough contact points to sample"

        # Sort by Z (axis=2)
        sorted_indices = np.argsort(-contact_positions[:, 2])
        sorted_positions = contact_positions[sorted_indices]

        # Choose evenly spaced indices
        step = (len(sorted_positions) - 1) / (k - 1)
        selected = [sorted_indices[int(round(i * step))] for i in range(k)]
        return selected

    def pick_distributed_groups(self, contact_positions, pos_tol, k):
        keys = np.round(contact_positions[:, 2] / pos_tol, decimals=0).astype(int)
        buckets = defaultdict(list)
        for i, key in enumerate(keys):
            buckets[key].append(i)

        group_indices = list(buckets.values())
        centroids = np.array([contact_positions[g].mean(axis=0) for g in group_indices])
        z_centroids = centroids[:, 2]

        sorted_group_ids = np.argsort(-z_centroids)

        if len(sorted_group_ids) < k:
            # Not enough groups - choose from all individual contact points
            rospy.logwarn("Not enough Z-groups; falling back to random sampling of individual points.")
            candidates = np.arange(len(contact_positions))
            return list(np.random.choice(candidates, size=k, replace=False))

        # Evenly spaced group selection across Z
        selected_group_ids = np.linspace(0, len(sorted_group_ids) - 1, k).astype(int)
        chosen_wp_indices = []

        for gid in sorted_group_ids[selected_group_ids]:
            candidates = group_indices[gid]
            chosen_wp_indices.append(np.random.choice(candidates))

        return chosen_wp_indices
    
    def match_contact_positions_by_z(self, contact_positions, contact_positions_original_A, num_trajectories, pos_tol: float = 0.01, centroid_tol: float = 0.01):
        contact_positions_original_A = np.array(contact_positions_original_A)

        # 1. Create Z-buckets
        keys = np.round(contact_positions[:, 2] / pos_tol).astype(int)
        buckets = defaultdict(list)
        for i, k in enumerate(keys):
            buckets[k].append(i)

        # 2. Compute centroids for each bucket
        group_indices = list(buckets.values())
        centroids = np.array([
            contact_positions[indices].mean(axis=0)
            for indices in group_indices
        ])
        centroid_z = centroids[:, 2]

        # 3. For each reference point, collect all indices from nearby centroids
        chosen_wp_indices = []
        for i in range(min(num_trajectories, len(contact_positions_original_A))):
            z_ref = contact_positions_original_A[i, 2]
            dists = np.abs(centroid_z - z_ref)
            close_centroid_idxs = np.where(dists <= centroid_tol)[0]

            if len(close_centroid_idxs) == 0:
                # fallback to absolute closest centroid
                closest_group_idx = np.argmin(dists)
                group = group_indices[closest_group_idx]
            else:
                # Combine all matching groups
                group = []
                for idx in close_centroid_idxs:
                    group.extend(group_indices[idx])

            chosen_wp_indices.append(np.random.choice(group))

        return chosen_wp_indices

    def save_rng_state(self):
        """Save the current state of the random number generator."""
        rng_state = np.random.get_state()
        with open(self.rng_state_path, 'wb') as f:
            pickle.dump(rng_state, f)
        rospy.loginfo("[FSM] Random number generator state saved.")

    def load_rng_state(self):
        """Load the saved state of the random number generator."""
        if os.path.exists(self.rng_state_path):
            with open(self.rng_state_path, 'rb') as f:
                rng_state = pickle.load(f)
            np.random.set_state(rng_state)
            rospy.loginfo("[FSM] Random number generator state loaded.")
        else:
            rospy.logwarn("[FSM] No saved random number generator state found. Using default state.")

    def run(self):
        """Run the FSM."""
        rospy.loginfo("[FSM] Starting FSM...")

        IS_OFFLINE = self.IS_OFFLINE
        CORRECT_ON_TOUCH = self.CORRECT_ON_TOUCH
        LOAD_SESSION = self.LOAD_SESSION

        is_offline_mode_list = []
        if IS_OFFLINE:
            is_offline_mode_list = [True, False]
            self.cabinet_filename = os.path.join(self.base_dir, f'offline_cabinets_estimation.csv')
            self.cabinet_filename_gt = os.path.join(self.base_dir, f'offline_cabinets_gt.csv')
            self.touches_filename = os.path.join(self.base_dir, f'offline_cabinets_touches.csv')
        else:
            is_offline_mode_list = [False]
            self.cabinet_filename = os.path.join(self.base_dir, f'online_cabinets_estimation.csv')
            self.cabinet_filename_gt = os.path.join(self.base_dir, f'online_cabinets_gt.csv')
            self.touches_filename = os.path.join(self.base_dir, f'online_cabinets_touches.csv')
        
        is_plan_traj = False if IS_OFFLINE else True
        is_new_scene = True
        init_loaded = False
        waypoint_trajectories_master = []
        trajectories_master = []
        if LOAD_SESSION:
            rospy.loginfo("[FSM] Loading session...")
            self.py_touch.set_visualization(False)
            scenes_done, trajs_done, self.idx_to_save, IS_OFFLINE_, is_plan_traj, is_new_scene, waypoint_trajectories_master, trajectories_master = self.load_session()
            self.py_touch.set_visualization(True)
            self.IS_OFFLINE = IS_OFFLINE_
            init_loaded = True
            is_offline_mode_list = [False]  if not self.IS_OFFLINE else is_offline_mode_list
        else:
            self.save_models_init_info_for_rvl()
            self.idx_to_save = -1
            self.touch_session_set = False
            trajs_done = 0
            scenes_done = 0

        # home_q = np.array([-np.pi/2, 0.0, -5/6*np.pi, -np.pi/6, np.pi/2, -3/4 * np.pi])
        self.home_q = np.array([-np.pi/2, -np.pi/2, 0.0, -np.pi/2, 0.0, -3/4 * np.pi])
        pos_tol = 1e-2
        centroid_tol = 3e-2

        for IS_OFFLINE in is_offline_mode_list:
            rospy.loginfo(f"[FSM] Running in {'offline' if IS_OFFLINE else 'online'} mode.")

            if IS_OFFLINE:
                num_offline_scenes = 3
                num_scenes = num_offline_scenes
                num_trajectories = 3
                # self.state = "FRONT_SURFACE_TOUCHES"
                self.state = "PLAN_TRAJECTORY"  # debug purpose
                is_plan_traj = True # debug purpose
                # self.state = "EXECUTE_APPROACH_PATH" # debug purpose
                self.opening_angle = -12.0
                num_traj_pts = 3
                # self.selected_poses = np.concatenate([self.offline_gt_poses[:num_offline_scenes], self.gt_poses], axis=0)
                self.selected_poses = self.offline_gt_poses
            else:
                num_scenes = 10
                num_trajectories = 1
                self.state = "PLAN_TRAJECTORY"
                num_traj_pts = 10
                self.opening_angle = -45.0
                self.selected_poses = self.gt_poses

            self.state = "PLAN_TRAJECTORY"  if is_plan_traj else "FRONT_SURFACE_TOUCHES"

            for i_scene in range(scenes_done, num_scenes):
                self.gt_idx = i_scene
                self.initialize_for_scene(i_scene, IS_OFFLINE=IS_OFFLINE)
                self.initialize(is_new_scene)

                # self.load_rng_state()  # Load the random number generator state

                self.scale_T_G_DD_all()

                contact_positions_original_A = self.get_all_contact_positions_A(pos_tol, k=num_trajectories) # contact positions in ref frame S_A - picked from all possible positions.

                if IS_OFFLINE:
                    self.opening_angle = self.state_angle - 5.0

                # Set touch scene
                if is_new_scene:
                    self.set_new_touch_scene(self.state_angle, self.state_angle_gt)
                    self.touch_session_set = True
                    self.update_model_from_touch()
                    is_new_scene = True

                # waypoint_trajectories = []
                # trajectories = []

                # if self.state == "PLAN_TRAJECTORY":
                #     # self.add_cabinet_model_to_scene(self.state_angle, scale_factor=1.0)

                #     waypoint_trajectories_master_, trajectories_master_ = self.plan_trajectory(num_traj_pts)
                #     if len(trajectories_master_) < 1:
                #         rospy.logerr("[FSM] No trajectories generated. Skipping.")
                #         waypoint_trajectories_master_ = copy.deepcopy(waypoint_trajectories_master)
                #         trajectories_master_ = copy.deepcopy(trajectories_master)

                #     waypoint_trajectories: List[Waypoint]
                #     waypoint_trajectories = copy.deepcopy(waypoint_trajectories_master_)
                #     trajectories = copy.deepcopy(trajectories_master_)

                # # contact_positions = []
                # # Select contact positions from the trajectories
                # contact_positions = self.get_contact_positions_A(waypoint_trajectories)

                # # keys = np.round(contact_positions[:, 2] / pos_tol, decimals=0).astype(int)  # Convert to integer keys for bucketing
                # # buckets = defaultdict(list)
                # # for i, k in enumerate(keys):
                # #     buckets[k].append(i)

                # # group_indices = list(buckets.values())
                # # centroids = np.array([contact_positions[g].mean(axis=0) for g in group_indices])
                
                # k = min(num_trajectories, contact_positions.shape[0])

                # # chosen_wp_indices = self.pick_z_distributed_indices(contact_positions=contact_positions, k=k)
                # if IS_OFFLINE:
                #     chosen_wp_indices = self.match_contact_positions_by_z(
                #         contact_positions=contact_positions,
                #         contact_positions_original_A=contact_positions_original_A, 
                #         num_trajectories=num_trajectories,
                #         pos_tol=pos_tol,
                #         centroid_tol=centroid_tol)
                #         # chosen_wp_indices = self.pick_distributed_groups(contact_positions=contact_positions, pos_tol=pos_tol, k=k)
                # else:
                #     # Online mode: pick uniformly random indices
                #     chosen_wp_indices = np.random.choice(np.arange(len(contact_positions)), size=k, replace=False).tolist()

                # selected_trajectories_wp = [waypoint_trajectories[i] for i in chosen_wp_indices]
                # selected_trajectories = [trajectories[i] for i in chosen_wp_indices]
                # contact_positions_A = [contact_positions[idx] for idx in chosen_wp_indices] # In ref frame S_A
        
                # self.state = "EXECUTE_APPROACH_PATH"
                self.state = "PLAN_TRAJECTORY"  
                trajectory = []
                trajectory_wp = []
                contact_poses = []

                for i_trajectory in range(trajs_done, num_trajectories):
                    # trajectory = selected_trajectories[i_trajectory]
                    # trajectory_wp = selected_trajectories_wp[i_trajectory]

                    states = np.linspace(self.state_angle, self.opening_angle, num_traj_pts)

                    # HOME POSE
                    # self.go_to_home_position()
                        
                    while self.state != "EXIT":
                        """
                        if self.state == "FRONT_SURFACE_TOUCHES":
                            state_angle = self.state_angle
                            self.add_cabinet_model_to_scene(state_angle, scale_factor=1.05)
                            s = np.array([self.door_thickness, self.s[0], self.s[1]])  # door thickness, width, height
                            T_TCP_A_all = sample_TCP_on_front_surface(s, edge_offset=0.06, step=0.03)
                            T_Arot_A = np.eye(4)
                            T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
                            self.cabinet_model.change_door_angle(state_angle)
                            cabinet_mesh = self.cabinet_model.create_mesh()
                            cabinet_mesh.transform(np.linalg.inv(self.cabinet_model.T_A_O))
                            T_Arot_W = self.cabinet_model.T_A_W @ T_Arot_A

                            feasible_points = filter_reachable_points(T_TCP_A_all, self.T_TCP_G, T_Arot_W, self.robot, self.rvl_manipulator, cabinet_mesh, self.gripper_mesh)
                            if len(feasible_points) < 1:
                                rospy.logerr("[FSM] No feasible points found on the front surface. Exiting.")
                                self.state = "EXIT"
                                continue
                            
                            rospy.loginfo(f"[FSM] Found {len(feasible_points)} feasible points on the front surface.")
                            points = np.array([fp[0][:3, 3] for fp in feasible_points])
                            indices = furthest_point_sampling(points, 3)
                            # samples structure:
                            #   T_TCP_A, T_TCP_A_approach, T_6_0_contact, T_6_0_approach, valid_joints_contact, valid_joints_approach
                            samples = [feasible_points[i] for i in indices]
                        
                            for sample in samples:
                                T_TCP_A, T_TCP_A_approach, T_6_0_contact, T_6_0_approach, joints, joints_approach = sample

                                T_Arot_W = self.cabinet_model.T_A_W @ T_Arot_A
                                T_Arot_0 = np.linalg.inv(self.robot.T_0_W) @ T_Arot_W
                                T_6_0_contact = T_Arot_0 @ T_TCP_A @ np.linalg.inv(self.T_TCP_G) @ np.linalg.inv(self.robot.T_G_6)
                                T_6_0_approach = T_Arot_0 @ T_TCP_A_approach @ np.linalg.inv(self.T_TCP_G) @ np.linalg.inv(self.robot.T_G_6)

                                # Get new joints for the contact pose
                                joints, sucess = self.robot.get_all_ik_solutions_rvl(T_6_0_contact, check_env_collisions=False, check_self_collisions=True)
                                if not sucess:
                                    rospy.logerr("[FSM] No IK solutions found for the contact pose. Skipping.")
                                    continue
                                joints_approach, sucess = self.robot.get_all_ik_solutions_rvl(T_6_0_approach, check_env_collisions=True, check_self_collisions=True)
                                if not sucess:
                                    rospy.logerr("[FSM] No IK solutions found for the approach pose. Skipping.")
                                    continue
                                indices = get_nearest_joints_pair_indices(joints, joints_approach)
                                # joints_approach, _ = get_nearest_joints(joints_approach, self.robot.get_current_joint_values())
                                # joints, _ = get_nearest_joints(joints, joints_approach)

                                q_current = self.robot.get_current_joint_values()
                                q_contact = joints[indices[0]]
                                q_approach = joints_approach[indices[1]]
                                # q_approach, q_contact, _ = get_smoothest_trajectory_from_3_pts(q_current, joints_approach, joints)

                                # Approach path
                                approach_trajectory = np.vstack((q_current, q_approach))
                                traj, plan_success = self.robot.plan_to_joint_goal2(q_approach)
                                if not plan_success:
                                    traj = approach_trajectory

                                traj_success, collision_joints = self.execute_with_monitoring_remember_joints(traj, force_threshold=20.0, max_velocity=1.0, max_acceleration=0.7)
                                if not traj_success:
                                    if len(collision_joints) > 0:
                                        collision_joints = np.array(collision_joints)
                                        assert collision_joints.shape[0] == 2, "Collision joints should contain exactly two points."
                                        T_6_0_before = self.robot.get_fwd_kinematics_moveit(collision_joints[0])
                                        T_6_0_collision = self.robot.get_fwd_kinematics_moveit(collision_joints[1])
                                        self.set_touch(T_6_0_collision, 
                                                np.array((T_6_0_before, T_6_0_collision)),
                                                touch_type=TouchType.UNWANTED_TOUCH,
                                                state_angle=state_angle)
                                    rospy.logerr("[FSM] Approach path execution failed.")
                                    continue

                                # Contact path
                                front_touch_trajectory = np.vstack((self.robot.get_current_joint_values(), q_contact))
                                success = self.execute_insertion_path(front_touch_trajectory, force_threshold=5.0)
                                if success:
                                    # There was no touch detected, so we assume it was a miss
                                    self.set_touch(T_6_0_contact, 
                                                np.array((T_6_0_approach, T_6_0_contact)),
                                                touch_type=TouchType.MISS,
                                                state_angle=state_angle)
                                    if CORRECT_ON_TOUCH:
                                        self.correct_touch_model()
                                
                                self.go_to_home_position()
                            
                            self.state = "PLAN_TRAJECTORY" if CORRECT_ON_TOUCH else "EXECUTE_APPROACH_PATH"
                        """

                        if self.state == "PLAN_TRAJECTORY":
                            # self.save_rng_state()  # Save the random number generator state

                            waypoint_trajectories_, trajectories_ = self.plan_trajectory(num_traj_pts)

                            if trajectories_.shape[0] < 1:
                                rospy.logerr("[FSM] No trajectories generated.")
                                # take a random trajectory that exists
                                # trajectory = trajectories_master[np.random.randint(0, trajectories_master.shape[0])]
                                # self.initialize(True)
                                # self.cabinet_model.change_door_angle(self.state_angle)
                                # self.T_D_S = self.cabinet_model.T_A_W @ self.cabinet_model.T_Arot_A @ self.cabinet_model.T_D_Arot
                                # # self.rvl_manipulator.reset_vn_environment(self.T_D_S)
                                # self.rvl_manipulator.clear()
                                # del self.rvl_manipulator
                                
                                # self.state = "EXECUTE_APPROACH_PATH"
                                self.state = "PLAN_TRAJECTORY" # plan again
                                # Skip this scene
                                break
                                # continue
                            trajectories = trajectories_

                            new_contact_positions_A = self.get_contact_positions_A(waypoint_trajectories_)

                            chosen_wp_indices = self.match_contact_positions_by_z(
                                contact_positions=new_contact_positions_A,
                                contact_positions_original_A=contact_positions_original_A, 
                                num_trajectories=num_trajectories,
                                pos_tol=pos_tol,
                                centroid_tol=centroid_tol)

                            selected_trajectories_wp = [waypoint_trajectories_[i] for i in chosen_wp_indices]
                            selected_trajectories = [trajectories[i] for i in chosen_wp_indices]

                            # # dists = np.abs(contact_positions_A[i_trajectory, 2] - new_contact_positions_A[:, 2])
                            # # rnd gaussian sampling
                            # candidates = np.arange(len(new_contact_positions_A))
                            # # Std dev is 0.2 / 3 = 0.066 - this is because the smallest door height is 0.2 and in 3rd std dev we cover 99.7% of the points
                            # # idx = self.gaussian_weighted_choice(candidates, dists, sigma=0.066)
                            # idx = self.gaussian_weighted_choice_fixed_scale(candidates, 
                            #                                                 contact_positions_original_A[i_trajectory][2], 
                            #                                                 new_contact_positions_A[:, 2], 
                            #                                                 door_height=self.cabinet_model.sz, 
                            #                                                 sigma_ratio=0.333)
                            # selected_trajectories_wp[i_trajectory] = waypoint_trajectories_[idx]
                            # selected_trajectories[i_trajectory] = trajectories[idx]
                            # contact_positions_A[i_trajectory] = new_contact_positions_A[idx] # In ref frame S_A
                            # # for other trajectories, find the closest contact position
                            # for j in range(i_trajectory + 1, num_trajectories):
                            #     dists = np.linalg.norm(new_contact_positions_A - contact_positions_A[j], axis=1)
                            #     idx = np.argmin(dists)
                            #     selected_trajectories_wp[j] = waypoint_trajectories_[idx]
                            #     selected_trajectories[j] = trajectories[idx]
                            #     contact_positions_A[j] = new_contact_positions_A[idx]

                            trajectory = selected_trajectories[i_trajectory]
                            waypoint_trajectory_ = selected_trajectories_wp[i_trajectory]
                            contact_poses = waypoint_trajectory_.get_full_trajectory_task_space()[2:, ...]

                            # T_6_0_traj = waypoint_trajectory_.get_full_trajectory_task_space()[:3, ...]
                            # q_traj = waypoint_trajectory_.get_full_trajectory_joint_space()[:3, ...]
                            # q_traj[:, 0] += np.pi  # Adjust the trajectory to match the RVL
                            # for i in range(3):
                            #     T_6_0_ = T_6_0_traj[i, ...]
                            #     T_G_0_ = T_6_0_ @ self.robot.T_G_6
                            #     self.rvl_manipulator.visualize_vn_current_state(q_traj[i], T_G_0_)

                            # self.go_to_home_position()

                            self.state = "EXECUTE_APPROACH_PATH"

                        elif self.state == "EXECUTE_APPROACH_PATH":
                            current_joints = self.robot.get_current_joint_values()
                            approach_trajectory = np.vstack((current_joints, trajectory[:2]))  # Take the first two points of the open trajectory
                            forces = [30.0, 10.0]
                            velocities = [1.0, 0.1]
                            accelerations = [0.7, 0.05]
                            
                            state_angle = self.state_angle
                            self.add_cabinet_model_to_scene(state_angle, scale_factor=1.05) 

                            T_6_0_collision = None
                            approach_success = True

                            for i_approach in range(2):
                                current_joints = self.robot.get_current_joint_values()
                                approach_trajectory = np.vstack((current_joints, trajectory[i_approach]))

                                plan_success = False
                                if i_approach < 1:
                                    planned_trajectory, plan_success = self.robot.plan_to_joint_goals2(approach_trajectory)
                                if not plan_success:
                                    self.zero_sensor() # Zero sensor before executing the trajectory
                                    planned_trajectory = approach_trajectory
                                success, collision_joints = self.execute_with_monitoring_remember_joints(planned_trajectory, force_threshold=forces[i_approach], max_velocity=velocities[i_approach], max_acceleration=accelerations[i_approach])
                                if not success:
                                    if len(collision_joints) > 0:
                                        T_6_0_contact = contact_poses[0, ...]
                                        
                                        collision_joints = np.array(collision_joints)

                                        # Set touch info
                                        T_6_0_before = self.robot.get_fwd_kinematics_moveit(collision_joints[0])
                                        T_6_0_collision = self.robot.get_fwd_kinematics_moveit(collision_joints[1])
                                        
                                        key = input("[FSM] Approach path execution failed. Press 1 if it is a touch and 0 if it should go again: ")

                                        if key != '1':
                                            rospy.logwarn("[FSM] Contact point is too far from the collision point. Skipping touch recording.")
                                            self.state = "EXECUTE_APPROACH_PATH"
                                            approach_success = False
                                            T_6_0_collision = None
                                            break
                                        else:
                                            self.set_touch(T_6_0_collision, 
                                                    np.array((T_6_0_before, T_6_0_collision)),
                                                    touch_type=TouchType.UNWANTED_TOUCH,
                                                    state_angle=state_angle)
                                    rospy.logerr("[FSM] Approach path execution failed.")
                                    approach_success = False
                                    break

                            if not approach_success:
                                # self.go_to_home_position()
                                if T_6_0_collision is not None:
                                    if CORRECT_ON_TOUCH:
                                        self.correct_touch_model()
                                    # self.state = "PLAN_TRAJECTORY" if not IS_OFFLINE else "EXIT"
                                    self.state = "PLAN_TRAJECTORY"
                                    continue
                            else:
                                self.state = "EXECUTE_INSERTION_PATH"

                        elif self.state == "EXECUTE_INSERTION_PATH":
                            rospy.loginfo("[FSM] Executing insertion path...")
                            current_joints = self.robot.get_current_joint_values()
                            insertion_trajectory = np.vstack((current_joints.copy(), trajectory[2]))

                            success, skipped_touch = self.execute_insertion_path(insertion_trajectory, force_threshold=7.0, state_angle=state_angle)
                            
                            if skipped_touch:
                                self.state = "EXECUTE_APPROACH_PATH"
                                rospy.logwarn("[FSM] Skipped touch during insertion path execution. Retrying approach path.")
                                continue
                            
                            if not success:
                                # self.state = "EXIT" if IS_OFFLINE else "PLAN_TRAJECTORY"
                                self.state = "PLAN_TRAJECTORY"
                            else:
                                rospy.loginfo("[FSM] Insertion path executed successfully.")
                                self.state = "EXECUTE_OPENING_PATH"


                        elif self.state == "EXECUTE_OPENING_PATH":
                            open_trajectory = trajectory[2:]

                            # Set up monitoring for tactile loss and contact establishment
                            tactile_loss_joints = []
                            self.robot.tactile_contact_established = False
                            monitor_thread = threading.Thread(
                                target=monitor_tactile_loss_and_remember_joints, 
                                args=(self.robot, tactile_loss_joints, 0.3, 4.0, 50.0) # robot, tactile_loss_joints, force_threshold, timeout, refresh_rate
                            )
                            tactile_establish_joints = []
                            monitor_thread2 = threading.Thread(
                                target=monitor_tactile_contact_establish,
                                args=(self.robot, tactile_establish_joints, 0.5, 4.0, 50.0) # robot, joints, threshold, timeout, refresh_rate
                            )
                            monitor_thread2.start()
                            monitor_thread.start()
                            success = self.robot.send_joint_trajectory_action2(open_trajectory, max_velocity=0.2, max_acceleration=0.2)
                            monitor_thread2.join()
                            monitor_thread.join()

                            if success:
                                if len(tactile_establish_joints) > 0:
                                    tactile_establish_joints = np.array(tactile_establish_joints)
                                    if tactile_establish_joints.shape[0] == 2:
                                        T_6before_0 = self.robot.get_fwd_kinematics_moveit(tactile_establish_joints[-2])
                                    elif tactile_establish_joints.shape[0] == 1:
                                        T_6before_0 = self.robot.get_fwd_kinematics_moveit(open_trajectory[0])
                                    else: 
                                        rospy.logerr("[FSM] Unexpected number of tactile establish joints. Exiting.")
                                        return

                                    T_6contact_0 = self.robot.get_fwd_kinematics_moveit(tactile_establish_joints[-1])
                                    
                                    # Calculate the angle based on the distance from the contact points
                                    closest_idx = self.get_closest_contact_pose_idx(contact_poses, T_6contact_0)
                                    state_angle = states[closest_idx]

                                    self.set_touch(T_6contact_0, 
                                                np.array((T_6before_0, T_6contact_0)),
                                                touch_type=TouchType.WANTED_TOUCH,
                                                state_angle=state_angle, only_save=True)
                                    
                                    T_6_0_current = self.robot.get_current_tool_pose()
                                    T_6_0_backup = T_6_0_current.copy()
                                    T_6_0_backup[:3, 3] += -0.03 * T_6_0_backup[:3, 2]
                                    backup_q = self.robot.get_closest_ik_solution(T_6_0_backup, self.robot.get_current_joint_values())
                                    if backup_q is not None:
                                        backup_traj = np.vstack((self.robot.get_current_joint_values(), backup_q))
                                        rospy.loginfo("[FSM] Executing backup trajectory...")
                                        self.execute_with_monitoring(backup_traj, force_threshold=30.0, max_velocity=0.1, max_acceleration=0.1)

                                    rospy.loginfo("[FSM] Opening path executed successfully.")
                                    self.state = "EXIT"
                                else:
                                    rospy.logwarn("[FSM] No tactile contact established during opening path execution.")
                                    T_6contact_0 = self.robot.get_fwd_kinematics_moveit(open_trajectory[0])
                                    T_TCPcontact_0 = T_6contact_0 @ self.robot.T_G_6 @ self.T_TCP_G
                                    T_D_0 = np.linalg.inv(self.robot.T_0_W) @ self.T_D_S

                                    T_TCPcontact_D = np.linalg.inv(T_D_0) @ T_TCPcontact_0
                                    T_TCPcontact_D[2, 3] = 0.0
                                    T_TCP_realcontact_0 = T_D_0 @ T_TCPcontact_D
                                    T_6realcontact_0 = T_TCP_realcontact_0 @ np.linalg.inv(self.T_TCP_G) @ np.linalg.inv(self.robot.T_G_6)
                                    self.set_touch(T_6realcontact_0, 
                                                np.array((T_6contact_0, T_6realcontact_0)),
                                                touch_type=TouchType.MISS,
                                                state_angle=self.state_angle)

                                    if CORRECT_ON_TOUCH:
                                        self.correct_touch_model()
                                    success = False
                                    self.state = "PLAN_TRAJECTORY"

                            else:
                                if len(tactile_loss_joints) > 0:
                                    tactile_loss_joints = np.array(tactile_loss_joints)
                                    if tactile_loss_joints.shape[0] == 2:
                                        T_6before_0 = self.robot.get_fwd_kinematics_moveit(tactile_loss_joints[-2])
                                    else:
                                        T_6before_0 = self.robot.get_fwd_kinematics_moveit(open_trajectory[0])
                                    T_6miss_0 = self.robot.get_fwd_kinematics_moveit(tactile_loss_joints[-1])
                                    
                                    T_TCPmiss_0 = T_6miss_0 @ self.T_TCP_6

                                    states_ = np.linspace(self.state_angle, self.opening_angle, 200)
                                    states_ = np.deg2rad(states_)
                                    T_Arot_A_arr = np.zeros((len(states_), 4, 4))
                                    T_Arot_A_arr[:, :3, :3] = rotz_multiple(states_)
                                    T_Arot_A_arr[:, 3, 3] = 1.0
                                    T_D_S_arr = self.cabinet_model.T_A_W[np.newaxis, ...] @ T_Arot_A_arr @ self.cabinet_model.T_D_Arot[np.newaxis, ...]

                                    T_TCPmiss_D_arr = np.linalg.inv(T_D_S_arr) @ T_TCPmiss_0
                                    t_TCPmiss_D_array = T_TCPmiss_D_arr[:, :3, 3]
                                    mask_ = t_TCPmiss_D_array[:, 2] > 0.0
                                    t_TCPmiss_D_array = t_TCPmiss_D_array[mask_]
                                    states_ = states_[mask_]
                                    if t_TCPmiss_D_array.shape[0] < 1:
                                        rospy.logerr("[FSM] No valid TCP contact points found in the door frame. Skipping touch recording.")
                                        self.state = "PLAN_TRAJECTORY"
                                        continue

                                    idx_ = np.argmin(t_TCPmiss_D_array[:, 2])
                                    state_angle = np.rad2deg(states_[idx_])
                                    self.state_angle = state_angle

                                    states_gt_ = np.linspace(self.state_angle_gt, self.opening_angle, 200)
                                    states_gt_ = np.deg2rad(states_gt_)
                                    T_Arot_A_arr_gt = np.zeros((len(states_gt_), 4, 4))
                                    T_Arot_A_arr_gt[:, :3, :3] = rotz_multiple(states_gt_)
                                    T_Arot_A_arr_gt[:, 3, 3] = 1.0
                                    T_D_S_arr_gt = self.cabinet_gt.T_A_W[np.newaxis, ...] @ T_Arot_A_arr_gt @ self.cabinet_gt.T_D_Arot[np.newaxis, ...]

                                    T_TCPmiss_D_arr_gt = np.linalg.inv(T_D_S_arr_gt) @ T_TCPmiss_0
                                    t_TCPmiss_D_array_gt = T_TCPmiss_D_arr_gt[:, :3, 3]
                                    mask_ = t_TCPmiss_D_array_gt[:, 2] > 0.0
                                    t_TCPmiss_D_array_gt = t_TCPmiss_D_array_gt[mask_]
                                    if t_TCPmiss_D_array_gt.shape[0] < 1:
                                        state_angle_gt = state_angle
                                    else:
                                        idx_gt = np.argmin(t_TCPmiss_D_array_gt[:, 2])
                                        states_gt_ = states_gt_[mask_]
                                        state_angle_gt = np.rad2deg(states_gt_[idx_gt])
                                    
                                    self.state_angle_gt = state_angle_gt

                                    self.set_new_touch_scene(state_angle, state_angle_gt)
                                    self.set_touch(T_6miss_0, 
                                                np.array((T_6before_0, T_6miss_0)),
                                                touch_type=TouchType.MISS,
                                                state_angle=state_angle)

                                    if CORRECT_ON_TOUCH:
                                        self.correct_touch_model()
                                
                                    # self.state = "RECAPTURE"
                                    # elif self.state == "RECAPTURE":
                                    self.recapture(trajectory)
                                else:
                                    rospy.logwarn("[FSM] No tactile contact established during opening path execution.")
                                    T_6contact_0 = self.robot.get_fwd_kinematics_moveit(open_trajectory[0])
                                    T_TCPcontact_0 = T_6contact_0 @ self.robot.T_G_6 @ self.T_TCP_G
                                    T_D_0 = np.linalg.inv(self.robot.T_0_W) @ self.T_D_S

                                    T_TCPcontact_D = np.linalg.inv(T_D_0) @ T_TCPcontact_0
                                    T_TCPcontact_D[2, 3] = 0.0
                                    T_TCP_realcontact_0 = T_D_0 @ T_TCPcontact_D
                                    T_6realcontact_0 = T_TCP_realcontact_0 @ np.linalg.inv(self.T_TCP_G) @ np.linalg.inv(self.robot.T_G_6)
                                    self.set_touch(T_6realcontact_0, 
                                                np.array((T_6contact_0, T_6realcontact_0)),
                                                touch_type=TouchType.MISS,
                                                state_angle=self.state_angle)

                                    if CORRECT_ON_TOUCH:
                                        self.correct_touch_model()
                                    success = False
                                    self.state = "PLAN_TRAJECTORY"

                    if i_trajectory < num_trajectories - 1:
                        trajectory_wp = selected_trajectories_wp[i_trajectory + 1]
                        trajectory = selected_trajectories[i_trajectory + 1]
                        contact_poses = trajectory_wp.get_full_trajectory_task_space()[2:, ...]

                    if IS_OFFLINE and self.state == "EXIT":
                        # Go to next trajectory, but change the state
                        rospy.loginfo("[FSM] Exiting current trajectory. Moving to next contact touch...")
                        self.state = "EXECUTE_APPROACH_PATH"
                        # self.state = "PLAN_TRAJECTORY"
                    
                    trajs_done = 0 # reset trajectory done counter for the next scene


                input("Press Enter to continue to the next scene...")
                is_new_scene = True
                self.state = "PLAN_TRAJECTORY"
            
            scenes_done = 0


if __name__ == "__main__":
    fsm = DoorReplanningFSM(gt_idx=0, IS_OFFLINE=False, CORRECT_ON_TOUCH=True, LOAD_SESSION=True)
    fsm.run()