import os
import numpy as np
from gazebo_push_open.cabinet_model import Cabinet
import open3d as o3d
import copy
import RVLPYDDManipulator as rvlpy

def main():
    # Define paths
    base_dir = "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection"
    RVL_data_dir = os.path.join(base_dir, "RVL_data")

    cabinets_estimation_path = os.path.join(RVL_data_dir, "cabinets_estimation.csv")
    cabinets_gt_path = os.path.join(RVL_data_dir, "cabinets_gt.csv")
    cabinets_touches_path = os.path.join(RVL_data_dir, "cabinets_touches.csv")

    # Gripper path
    gripper_mesh_path = os.path.join('/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply')
    T_G_6 = np.load(os.path.join(RVL_data_dir, 'T_G_6.npy'))

    gripper_mesh = o3d.io.read_triangle_mesh(gripper_mesh_path)

    # Load CSV files
    cabinets_estimation = np.genfromtxt(cabinets_estimation_path, delimiter=',', skip_header=1)
    cabinets_estimation = np.atleast_2d(cabinets_estimation)
    cabinets_gt = np.genfromtxt(cabinets_gt_path, delimiter=',', skip_header=1)
    cabinets_gt = np.atleast_2d(cabinets_gt)
    cabinets_touches = np.genfromtxt(cabinets_touches_path, delimiter=',', skip_header=1)
    cabinets_touches = np.atleast_2d(cabinets_touches)
    
    static_d = 0.4

    # RVL initialization
    rvl_ddmanipulator_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
    rvl_touch_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Touch_Cupec.cfg'
    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_ddmanipulator_cfg)

    rvl_touch = rvl_manipulator.py_touch
    rvl_touch.create(rvl_touch_cfg)

    T_tool_6 = np.load(os.path.join(RVL_data_dir, 'T_tool_6.npy'))
    a_tool = 0.019
    b_tool = 0.064
    c_tool = 0.007
    d_tool = 0.049
    h_tool = 0.02706
    rvl_touch.create_simple_tool(a_tool, b_tool, c_tool, d_tool, h_tool, T_tool_6)

    camera_fu = 597.9033203125
    camera_fv = 598.47998046875
    camera_uc = 323.8436584472656
    camera_vc = 236.32774353027344
    camera_w = 640
    camera_h = 480
    rvl_touch.set_camera_params(camera_fu, camera_fv, camera_uc, camera_vc, camera_w, camera_h)

    touch_a = static_d
    touch_b = 0.0
    touch_c = 0.005


    # Extract estimated cabinets
    for i_cabinet, cabinet_ in enumerate(cabinets_estimation):
        idx_ = cabinet_[0]

        # Estimated cabinet
        s = cabinet_[1:4]
        r = cabinet_[4:6]
        state_angle = cabinet_[6]
        R_A_C = cabinet_[7:16].reshape(3, 3)
        t_A_C = cabinet_[16:19]
        R_C_E = cabinet_[19:28].reshape(3, 3)
        t_C_E = cabinet_[28:31]
        R_E_0 = cabinet_[31:40].reshape(3, 3)
        t_E_0 = cabinet_[40:43]

        T_A_C = np.eye(4)
        T_A_C[:3, :3] = R_A_C
        T_A_C[:3, 3] = t_A_C

        T_C_E = np.eye(4)
        T_C_E[:3, :3] = R_C_E
        T_C_E[:3, 3] = t_C_E

        # Unscale the transform matrix
        R_C_E = T_C_E[:3, :3].copy()
        R_E_C = R_C_E.T
        I_ = R_C_E @ R_E_C
        mean_trace = np.trace(I_) / 3.0
        scale_factor = np.sqrt(mean_trace)

        # Scale the matrices
        T_C_E[:3, :3] /= scale_factor
        T_A_C[:3, 3] *= scale_factor

        T_E_0 = np.eye(4)
        T_E_0[:3, :3] = R_E_0
        T_E_0[:3, 3] = t_E_0

        T_A_0 = T_E_0 @ T_C_E @ T_A_C
        cabinet_est = Cabinet(np.array([s[1], s[2], s[0], static_d]), 
                              r,
                              axis_pos=-1,
                              T_A_S=np.eye(4),
                              save_path=None,
                              has_handle=False)
        cabinet_est.change_door_angle(state_angle)
        cabinet_est_mesh = cabinet_est.create_mesh()
        cabinet_est_mesh.transform(np.linalg.inv(cabinet_est.T_A_O_init))
        cabinet_est_mesh.transform(T_A_0)

        # Ground truth cabinet
        cabinet_gt_ = cabinets_gt[i_cabinet]
        idx_gt = cabinet_gt_[0]
        s_gt = cabinet_gt_[1:4]
        r_gt = cabinet_gt_[4:6]
        state_angle_gt = cabinet_gt_[6]
        R_A_0_gt = cabinet_gt_[7:16].reshape(3, 3)
        t_A_0_gt = cabinet_gt_[16:19]
        T_A_0_gt = np.eye(4)
        T_A_0_gt[:3, :3] = R_A_0_gt
        T_A_0_gt[:3, 3] = t_A_0_gt

        cabinet_gt = Cabinet(np.array([s_gt[1], s_gt[2], s_gt[0], static_d]),
                             r_gt,
                             axis_pos=-1,
                             T_A_S=np.eye(4),
                             save_path=None,
                             has_handle=False)
        cabinet_gt.change_door_angle(state_angle_gt)
        cabinet_gt_mesh = cabinet_gt.create_mesh()
        cabinet_gt_mesh.transform(np.linalg.inv(cabinet_gt.T_A_O_init))
        cabinet_gt_mesh.paint_uniform_color([0.2, 0.5, 0.2])
        cabinet_gt_mesh.transform(T_A_0_gt)

        # PYTouch set cabinet params
        rvl_touch.set_cabinet_params(s[0], s[1], s[2], r[0], r[1], touch_a, touch_b, touch_c, state_angle)
        rvl_touch.set_scene_poses(T_C_E, T_A_C, T_E_0)
        rvl_touch.reset_touches()
        touch_is_init = True


        # Extract touches
        touches = cabinets_touches[cabinets_touches[:, 0] == idx_]

        touches_meshes = []
        for touch in touches:
            cabinet_touch_idx_ = int(touch[0])
            R_Ek_E = touch[1:10].reshape(3, 3)
            t_Ek_E = touch[10:13]
            T_6k_6 = np.eye(4)
            T_6k_6[:3, :3] = R_Ek_E
            T_6k_6[:3, 3] = t_Ek_E
            V_ = touch[13:16]

            T_G_0 = T_E_0 @ T_6k_6 @ T_G_6
            # t_ = T_G_0[:3, 2] * 0.02
            # T_G_0[:3, 3] += t_
            touch_mesh_ = copy.deepcopy(gripper_mesh)
            touch_mesh_.transform(T_G_0)
            touch_mesh_.paint_uniform_color([0.8, 0.2, 0.2])
            touch_mesh_.compute_vertex_normals()
            # touches_meshes.append(touch_mesh_)

            # debug
            # T_6contact_6capture = np.load('/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/RVL_data/T_6contact_6capture.npy')
            # T_6contact_0 = T_E_0 @ T_6contact_6capture
            T_6contact_0 = T_E_0 @ T_6k_6
            T_Gcontact_0 = T_6contact_0 @ T_G_6
            contact_mesh_ = copy.deepcopy(gripper_mesh)
            contact_mesh_.transform(T_Gcontact_0)
            contact_mesh_.paint_uniform_color([0.2, 0.2, 0.2])
            contact_mesh_.compute_vertex_normals()
            touches_meshes.append(contact_mesh_)

            # RVL touch
            rvl_touch.set_touch(T_6k_6, V_)

            # Test correction
            rvl_touch.test_correction(touch_is_init)
            touch_is_init = False
            rvl_manipulator.set_environment_from_touch()

            T_A_E_corrected = rvl_manipulator.get_corrected_cabinet_pose()
            T_C_E_corrected = rvl_manipulator.get_corrected_camera_pose()

            T_G_0 = T_E_0 @ T_6k_6 @ T_G_6
            q = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
            # rvl_manipulator.visualize_current_state(q, T_G_0)
            rvl_manipulator.visualize_vn_model()

        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])

        # Visualize estimated and ground truth cabinets
        o3d.visualization.draw_geometries([cabinet_est_mesh, cabinet_gt_mesh, origin_frame] + touches_meshes,
                                          window_name=f"Cabinet {idx_} Estimation vs GT",
                                          mesh_show_back_face=True)
        

if __name__ == "__main__":
    main()