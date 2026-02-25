import numpy as np
import open3d as open3d

def save_camera_parameters(camera_parameters, filename):
    open3d.io.write_pinhole_camera_parameters(filename, camera_parameters)

tool_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.ply')
tool_mesh.compute_vertex_normals()

vis = open3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(tool_mesh)

vis.run()

view_control = vis.get_view_control()
params = view_control.convert_to_pinhole_camera_parameters()
save_camera_parameters(params, '/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/o3d_camera_params.json')
# view_matrix  = view_control.convert_to_pinhole_camera_parameters().extrinsic

# Print the view matrix
# print("View Matrix:")
# print(view_matrix)

# np.save('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/camera_view_matrix.npy', view_matrix)


# Destroy the visualizer window
vis.destroy_window()