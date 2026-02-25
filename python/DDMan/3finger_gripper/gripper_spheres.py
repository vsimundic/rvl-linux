import numpy as np
import open3d

# [center.x, center.y, center.z, R] w.r.t. origin of gripper
spheres = np.matrix(
[[-8.2300e+01, -8.2000e+00,  9.3667e+01,  1.2500e+01],
 [-8.2300e+01,  8.2000e+00,  9.3667e+01,  1.2500e+01],
 [-8.6828e+01,  8.2000e+00,  8.1167e+01,  1.2500e+01],
 [-8.6828e+01, -8.2000e+00,  8.1167e+01,  1.2500e+01],
 [-8.5828e+01,  0.0000e+00,  6.7000e+01,  2.0000e+01],
 [-7.5547e+01,  2.3100e-07,  4.2000e+01,  2.6000e+01],
 [-5.6625e+01,  7.3620e-07,  9.8120e+00,  3.0250e+01],
 [ 0.0000e+00,  0.0000e+00, -5.6000e+01,  1.0300e+02],
 [ 5.6600e+01, -2.6500e+01,  6.8000e+00,  3.9000e+01],
 [ 5.6600e+01,  2.6500e+01,  6.8000e+00,  3.9000e+01],
 [ 7.5000e+01, -2.1500e+01,  4.0000e+01,  2.8250e+01],
 [ 7.5000e+01,  2.1500e+01,  4.0000e+01,  2.8250e+01],
 [ 8.5000e+01, -1.6000e+01,  6.0800e+01,  2.5000e+01],
 [ 8.5000e+01,  1.6000e+01,  6.0800e+01,  2.5000e+01],
 [ 8.3000e+01,  2.1000e+01,  9.7000e+01,  9.5000e+00],
 [ 8.3500e+01,  1.0000e+01,  9.7000e+01,  9.5000e+00],
 [ 8.3500e+01,  0.0000e+00,  9.7000e+01,  9.5000e+00],
 [ 8.3500e+01, -1.0000e+01,  9.7000e+01,  9.5000e+00],
 [ 8.3000e+01, -2.1000e+01,  9.7000e+01,  9.5000e+00],
 [ 8.6000e+01,  2.5500e+01,  8.4000e+01,  1.2500e+01],
 [ 8.6000e+01,  1.2500e+01,  8.4000e+01,  1.2500e+01],
 [ 8.6000e+01,  0.0000e+00,  8.4000e+01,  1.2500e+01],
 [ 8.6000e+01, -1.2500e+01,  8.4000e+01,  1.2500e+01],
 [ 8.6000e+01, -2.5500e+01,  8.4000e+01,  1.2500e+01]]
)

# np.save('/home/RVLuser/rvl-linux/data/Robotiq3Finger/spheres.npy', spheres)
spheres /= 1000.

# with open('gripper_spheres.npy', 'rb') as f:
#     print(np.load(f))

# print(spheres[0,3])

sphere_meshes = []
sphere_meshes_ls = []
tool_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.ply')
tool_mesh_ls = open3d.geometry.LineSet.create_from_triangle_mesh(tool_mesh)
# tool_mesh.compute_vertex_normals()

for i in range(spheres.shape[0]):
    sphere = open3d.geometry.TriangleMesh.create_sphere(radius=spheres[i, 3])
    # sphere.compute_vertex_normals()
    sphere_ls = open3d.geometry.LineSet.create_from_triangle_mesh(sphere)

    # sphere_ls.compute_vertex_normals()
    vec = np.array(spheres[i, 0:3]).ravel()
    
    sphere.translate(vec)
    sphere_ls.translate(vec)

    sphere_meshes.append(sphere)
    sphere_meshes_ls.append(sphere_ls)

    tool_mesh+=sphere

print(len(sphere_meshes))
# sphere_meshes.append(tool_mesh)
sphere_meshes_ls.append(tool_mesh)
print(len(sphere_meshes))
# open3d.visualization.draw_geometries(sphere_meshes)
open3d.visualization.draw_geometries(sphere_meshes_ls)

tool_mesh.compute_vertex_normals()
open3d.io.write_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_spheres.dae', tool_mesh)

