import open3d as o3d
import numpy as np

origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
origin2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0.1])
origin.paint_uniform_color([0, 0, 1])
TAC = np.load('/home/RVLuser/ur5_ws/src/ao_manipulation/config/TAC_test.npy')
tac_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
tac_frame.transform(TAC)

o3d.visualization.draw_geometries([origin, tac_frame, origin2])