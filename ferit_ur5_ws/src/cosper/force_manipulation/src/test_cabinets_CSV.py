import os
import numpy as np
from gazebo_push_open.cabinet_model import Cabinet
import open3d as o3d
import copy

def main():
    # Define paths
    IS_OFFLINE = True
    base_dir = "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection"
    RVL_data_dir = os.path.join(base_dir, "RVL_data")

    cabinets_estimation_path = os.path.join(base_dir, "cabinets_estimation.csv" if not IS_OFFLINE else "offline_cabinets_estimation.csv")
    cabinets_gt_path = os.path.join(base_dir, "cabinets_gt.csv" if not IS_OFFLINE else "offline_cabinets_gt.csv")
    cabinets_touches_path = os.path.join(base_dir, "cabinets_touches.csv" if not IS_OFFLINE else "offline_cabinets_touches.csv")

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

    gt_meshes = []
    est_meshes = []

    # Extract estimated cabinets
    for i_cabinet, cabinet_ in enumerate(cabinets_estimation):
        idx_ = cabinet_[0]
        pose_idx = int(cabinet_[1])

        # Estimated cabinet
        s = cabinet_[2:5]
        r = cabinet_[5:7]
        state_angle = cabinet_[7]
        R_A_C = cabinet_[8:17].reshape(3, 3)
        t_A_C = cabinet_[17:20]
        R_C_E = cabinet_[20:29].reshape(3, 3)
        t_C_E = cabinet_[29:32]
        R_E_0 = cabinet_[32:41].reshape(3, 3)
        t_E_0 = cabinet_[41:44]

        T_A_C = np.eye(4)
        T_A_C[:3, :3] = R_A_C
        T_A_C[:3, 3] = t_A_C
        T_C_E = np.eye(4)
        T_C_E[:3, :3] = R_C_E
        T_C_E[:3, 3] = t_C_E

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

        T_A_E = T_C_E @ T_A_C
        cabinet_est_mesh.transform(T_A_E)

        est_meshes.append(cabinet_est_mesh)

        # Ground truth cabinet
        cabinet_gt_ = cabinets_gt[i_cabinet]
        idx_gt = cabinet_gt_[0]
        pose_idx_gt = int(cabinet_gt_[1])
        s_gt = cabinet_gt_[2:5]
        r_gt = cabinet_gt_[5:7]
        state_angle_gt = cabinet_gt_[7]
        R_A_0_gt = cabinet_gt_[8:17].reshape(3, 3)
        t_A_0_gt = cabinet_gt_[17:20]
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
        cabinet_gt_mesh = copy.deepcopy(cabinet_gt.create_mesh())
        cabinet_gt_mesh.transform(np.linalg.inv(cabinet_gt.T_A_O_init))
        cabinet_gt_mesh.paint_uniform_color([0.2, 0.5, 0.2])

        T_A_E_gt = np.linalg.inv(T_E_0) @ T_A_0_gt
        cabinet_gt_mesh.transform(T_A_E_gt)
        # cabinet_gt_mesh.transform(T_A_0_gt)
        gt_meshes.append(cabinet_gt_mesh)

        # Extract touches
        touches = cabinets_touches[cabinets_touches[:, 0] == idx_]

        touches_meshes = []
        for touch in touches:
            cabinet_touch_idx_ = int(touch[0])
            cabinet_touch_pose_idx = int(touch[1])

            if pose_idx != cabinet_touch_pose_idx:
                continue

            touch_type = touch[2]
            R_Ek_E = touch[3:12].reshape(3, 3)
            t_Ek_E = touch[12:15]
            T_6k_6 = np.eye(4)
            T_6k_6[:3, :3] = R_Ek_E
            T_6k_6[:3, 3] = t_Ek_E
            V_ = touch[15:18]

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
            T_G_6contact = T_6k_6 @ T_G_6
            contact_mesh_.transform(T_G_6contact)
            # contact_mesh_.transform(T_Gcontact_0)

            contact_mesh_.paint_uniform_color([0.2, 0.2, 0.2])
            contact_mesh_.compute_vertex_normals()
            touches_meshes.append(contact_mesh_)

        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])

        # Visualize estimated and ground truth cabinets
        # o3d.visualization.draw_geometries([cabinet_est_mesh, cabinet_gt_mesh, origin_frame] + touches_meshes,
        #                                   window_name=f"Cabinet {idx_} Estimation vs GT",
        #                                   mesh_show_back_face=True)
    o3d.visualization.draw_geometries(est_meshes + gt_meshes + [origin_frame],
                                      window_name=f"Cabinet {idx_} GT",
                                      mesh_show_back_face=True)

if __name__ == "__main__":
    main()