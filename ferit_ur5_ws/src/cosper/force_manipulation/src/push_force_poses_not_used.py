import numpy as np
from DDMan.push import door_model, tool_model
import open3d as o3d
import copy

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
    top_offset = 0.02
    side_offset = 0.02
    z_offset = 0.01

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

if __name__ == "__main__":
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