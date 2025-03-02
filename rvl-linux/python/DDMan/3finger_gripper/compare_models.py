import numpy as np
import open3d


# [center.x, center.y, center.z, R] w.r.t. origin of gripper

spheres = np.matrix(
[
# Solo finger
 [-82.300, -5.2000,  94.5,  12.500],
 [-82.300,  5.2000,  94.5,  12.500],
 [-86.828,  8.2000,  81.167,  12.500],
 [-86.828, -8.2000,  81.167,  12.500],

 [-90.828,  0.0000,  62.000,  23.000],

 [-80.547,  2.3100e-07,  36.000,  27.000],

 [-66.625,  7.3620e-07,  1.8120,  32],

    # Palm
 [ 00.000,  0.0000, -56.000,  85.00],

# Double fingers
 [ 61.5, -27.500,  -9,  40.000],
 [ 61.5,  27.500,  -9,  40.000],

 [ 83, -26,  36.000,  30],
 [ 83,  26,  36.000,  30],

 [ 88.000,  -21.500,    65.500,  22.000],
 [ 89.500,  00.000,     66.000,  20.000],
 [ 88.000,  21.500,     65.500,  22.000],

 [ 85,  26.5,  85.800,  11.00],
 [ 85.500,  11.750,  85.800,  11.00],
 [ 85.500,  0.0000,  85.800,  11.00],
 [ 85.500, -11.750,  85.800,  11.00],
 [ 85, -26.5,  85.800,  11.00],

 [ 82.000,  20,  97,  9.5000],
 [ 82.000,  10,  95.700,  9.5000],
 [ 82.000,  0,  95.700,  9.5000],
 [ 82.000, -10,  95.700,  9.5000],
 [ 82.000, -20,  97,  9.5000]
 ]
)

# spheres = np.matrix(
# [[-82.300, -8.2000,  93.667,  12.500],
#  [-82.300,  8.2000,  93.667,  12.500],
#  [-86.828,  8.2000,  81.167,  12.500],
#  [-86.828, -8.2000,  81.167,  12.500],

#  [-85.828,  0.0000,  66.000,  22.000],

#  [-75.547,  2.3100e-07,  42.000,  26.000],

#  [-56.625,  7.3620e-07,  9.8120,  30.250],
#  [ 00.000,  0.0000, -56.000,  103.00],
#  [ 56.600, -26.500,  6.8000,  39.000],
#  [ 56.600,  26.500,  6.8000,  39.000],
#  [ 75.000, -21.500,  40.000,  28.250],
#  [ 75.000,  21.500,  40.000,  28.250],

#  [ 86.000,  -21.500,    66.000,  20.000],
#  [ 86.500,  00.000,     66.000,  20.000],
#  [ 86.000,  21.500,     66.000,  20.000],

#  [ 82.000,  20.500,  95.700,  9.5000],
#  [ 82.000,  09.500,  95.700,  9.5000],
#  [ 82.000,  00.500,  95.700,  9.5000],
#  [ 82.000, -09.500,  95.700,  9.5000],
#  [ 82.000, -20.500,  95.700,  9.5000],

#  [ 85.500,  23.500,  85.800,  11.00],
#  [ 85.500,  11.750,  85.800,  11.00],
#  [ 85.500,  0.0000,  85.800,  11.00],
#  [ 85.500, -11.750,  85.800,  11.00],
#  [ 85.500, -23.500,  85.800,  11.00]]
# )

spheres2 = np.matrix([
    [-82.602, -9.30, 93.667, 11.6],
    [-82.609, 9.30, 93.667, 11.6],
    [-82.609, -0.135, 93.667, 11.6],
    [-86.828, 9.30, 81.167, 11.6],
    [-86.828, -9.30, 81.167, 11.6],
    [-86.828, -0.135, 81.167, 11.6],
    [-85.828, -9.00, 65.253, 17.00],
    [-85.828, 9.00, 65.253, 17.00],    
    [-75.547, 2.310E-07, 40.673, 26.00],
    [-56.625, 7.362E-07, 9.812, 30.25],
    [0.00, 0.00, -56.00, 103.00],
    [5+56.60, -26.50, 6.80, 39.00 +2],
    [5+56.60, 26.50, 6.80, 39.00 +2],
    [5+75.00, -21.50, 40.00, 28.25 +2],
    [5+75.00, 21.50, 40.00, 28.25 +2],
    [5+84.80, -16.00, 60.30, 24.5 +2],
    [5+84.80, 16.00, 60.30, 24.5 +2],
    [5+82.609, -0.135, 93.667, 13.75 +2],
    [5+82.609, 18.865, 93.667, 13.75 +2],
    [5+82.609, -19.00, 93.667, 13.75 +2],
    [5+86.609, -0.165, 81.667, 14.5 +2],
    [5+86.609, 21.865, 81.667, 14.5 +2],
    [5+86.609, -21.865, 81.667, 14.5 +2]
])

# np.save('/home/RVLuser/rvl-linux/data/Robotiq3Finger/spheres.npy', spheres)
# np.save('/home/RVLuser/rvl-linux/data/Robotiq3Finger/spheres2.npy', spheres2)
# with open('/home/RVLuser/rvl-linux/data/Robotiq3Finger/spheres.npy', 'wb') as f:
#     np.save(f, spheres)
spheres /= 1000.
spheres2 /= 1000.

sphere_meshes_ls2 = []
for i in range(spheres2.shape[0]):
    sphere = open3d.geometry.TriangleMesh.create_sphere(radius=spheres2[i, 3])
    # sphere.compute_vertex_normals()
    sphere_ls = open3d.geometry.LineSet.create_from_triangle_mesh(sphere)

    # sphere_ls.compute_vertex_normals()
    vec = np.array(spheres2[i, 0:3]).ravel()
    
    sphere.translate(vec)
    sphere_ls.translate(vec)

    # sphere_meshes.append(sphere)
    sphere_meshes_ls2.append(sphere_ls)

    # tool_mesh+=sphere



# with open('gripper_spheres.npy', 'rb') as f:
#     print(np.load(f))

# print(spheres[0,3])

sphere_meshes = []
sphere_meshes_ls = []

# tool_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.ply')
# tool_mesh_ls = open3d.geometry.LineSet.create_from_triangle_mesh(tool_mesh)
# # tool_mesh.compute_vertex_normals()
tool_mesh_ = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.ply')

for i in range(spheres.shape[0]):
    sphere = open3d.geometry.TriangleMesh.create_sphere(radius=spheres[i, 3])
    # sphere.compute_vertex_normals()
    sphere_ls = open3d.geometry.LineSet.create_from_triangle_mesh(sphere)
    sphere_ls.paint_uniform_color([0.8, 0, 0])
    # sphere_ls.compute_vertex_normals()
    vec = np.array(spheres[i, 0:3]).ravel()
    
    sphere.translate(vec)
    sphere_ls.translate(vec)

    sphere_meshes.append(sphere)
    sphere_meshes_ls.append(sphere_ls)

    tool_mesh_+=sphere

# open3d.io.write_triangle_mesh('/home/RVLuser/rvl-linux/data/Robotiq3Finger/mesh.ply', tool_mesh_)
# print(len(sphere_meshes))
# # sphere_meshes.append(tool_mesh)
# sphere_meshes_ls.append(tool_mesh)
# print(len(sphere_meshes))
# # open3d.visualization.draw_geometries(sphere_meshes)
# open3d.visualization.draw_geometries([tool_mesh])

# tool_mesh.compute_vertex_normals()
# open3d.io.write_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_spheres.dae', tool_mesh)




# COMPARE MODELS

origin_mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01)


T_3f_G = np.load('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/T_3f_G.npy')
T_3f_G[2, 3] -= 0.013
tool_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.ply')
tool_mesh.compute_vertex_normals()
tool_mesh_ls = open3d.geometry.LineSet.create_from_triangle_mesh(tool_mesh)
gripper_3f_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq-3f-gripper_articulated.stl')
gripper_3f_mesh.transform(T_3f_G)
# sphere_meshes_ls.append(tool_mesh)
sphere_meshes_ls.append(gripper_3f_mesh)
# sphere_meshes_ls.append(origin_mesh)
# sphere_meshes_ls.extend(sphere_meshes_ls2)

# gripper_3f_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/data/Robotiq3Finger/mesh.ply')
# sphere_meshes_ls.append(gripper_3f_mesh)


open3d.visualization.draw_geometries(sphere_meshes_ls)