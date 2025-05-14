#!/usr/bin/env python

import numpy as np
import RVLPYDDDetector

class DoorStateDetector:
    def __init__(self, detector_config_path, best_hyp_path):
        self.detector_config_path = detector_config_path
        self.best_hyp_path = best_hyp_path

        # Initialize detector
        self.detector = RVLPYDDDetector.PYDDDetector()
        self.detector.create(self.detector_config_path)
        self.detector.load_best_hypothesis(self.best_hyp_path)

    def detect_state(self, rgb_path, ply_path, T_Cdet_Cdiff):
        # # Load door models
        # with open(door_model_path, 'r') as f:
        #     door_model = json.load(f)
        # with open(diff_door_model_path, 'r') as f:
        #     diff_door_model = json.load(f)

        # # Compute transformations
        # q_detected = np.array(door_model["joint_values"])
        # T_6_0_detected = robot.forward_kinematics(q_detected)
        # T_C_0_detected = T_6_0_detected @ self.T_C_6

        # q_diff = np.array(diff_door_model["joint_values"])
        # T_6_0_diff = robot.forward_kinematics(q_diff)
        # T_C_0_diff = T_6_0_diff @ self.T_C_6

        # T_Cdet_Cdiff = np.linalg.inv(T_C_0_diff) @ T_C_0_detected

        # Add mesh and RGB image to the detector
        self.detector.add_mesh(ply_path)
        self.detector.add_rgb(rgb_path)

        # Recognize door state
        states = self.detector.recognize_ao_state(T_Cdet_Cdiff, True)
        return np.rad2deg(np.array(states).reshape(-1))