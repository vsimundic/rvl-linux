import numpy as np
import open3d


# [center.x, center.y, center.z, R] w.r.t. origin of gripper
spheres = np.matrix([
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
    [56.60, -26.50, 6.80, 39.00],
    [56.60, 26.50, 6.80, 39.00],
    [75.00, -21.50, 40.00, 28.25],
    [75.00, 21.50, 40.00, 28.25],
    [84.80, -16.00, 60.30, 24.5],
    [84.80, 16.00, 60.30, 24.5],
    [82.609, -0.135, 93.667, 13.75],
    [82.609, 18.865, 93.667, 13.75],
    [82.609, -19.00, 93.667, 13.75],
    [86.609, -0.165, 81.667, 14.5],
    [86.609, 21.865, 81.667, 14.5],
    [86.609, -21.865, 81.667, 14.5]
])

# with open('gripper_spheres.npy', 'wb') as f:
#     np.save(f, spheres)

# with open('gripper_spheres.npy', 'rb') as f:
#     print(np.load(f))

# print(spheres[0,3])

sphere_meshes = []
for i in range(spheres.shape[0]):
    sphere = open3d.geometry.TriangleMesh.create_sphere(radius=spheres[i, 3])
    vec = np.array(spheres[i, 0:3]).ravel()

    sphere.translate(vec)

    sphere_meshes.append(sphere)

tool_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.stl')

sphere_meshes.append(tool_mesh)
open3d.visualization.draw_geometries(sphere_meshes)