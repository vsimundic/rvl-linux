import open3d as o3d
import numpy as np

def create_cylinder(radius, height, resolution=50, ):
    """
    Create a cylinder mesh aligned along the specified axis.
    
    Args:
        radius (float): Radius of the cylinder.
        height (float): Height of the cylinder.
        resolution (int): Number of radial segments.
        orientation (str): 'x', 'y', or 'z' to specify cylinder alignment.
    
    Returns:
        o3d.geometry.TriangleMesh: The cylinder mesh.
    """
    # Create a cylinder aligned along the Z-axis by default
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height, resolution=resolution)
    
    # # Move to center at (0,0,0)
    # cylinder.translate((0, 0, -height / 2))

    # Set color
    cylinder.paint_uniform_color([0.7, 0.2, 0.2])  # Red
    
    return cylinder

# Define Cylinder Parameters
radius1 = 0.06
height1 = 0.23

# Create Cylinders
cylinder1 = create_cylinder(radius1, height1) 

T_c1 = np.eye(4)
T_c1[:3, :3] = np.array([[0.0227533989,-0.995899498,0.087558426],
                         [-0.250479907,-0.090466544,-0.963885665],
                         [0.967854321,4.37113883e-08,-0.251511276]]) 
T_c1[:3, 3] = np.array([-0.0388654247, 0.427849621, 0.505166471])
cylinder1.transform(T_c1)
cylinder1.compute_vertex_normals()

radius2 = 0.038
height2 = 0.293
cylinder2 = create_cylinder(radius2, height2) 
tool_mesh = o3d.io.write_triangle_mesh('/home/RVLuser/rvl-linux/data/Robotiq3Finger/mesh.ply')
T_c2 = np.eye(4)

T_c2[:3, :3] = np.array([0.0598633774,-0.0357743539,0.997565329,-0.856310844,0.511730671,0.0697382838,-0.512979627,-0.858400822,-4.37113883e-08]).reshape((3,3))
T_c2[:3, 3] = np.array([-0.0277478825,0.396917582,0.71594882])


cylinder2.transform(T_c2)
cylinder2.compute_vertex_normals()

tool_mesh = o3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/data/Robotiq3Finger/mesh.ply')
T_tool = np.eye(4)
T_tool[:3, :3] = np.array([[0.707106769,0.707106769,0],
                            [0,0,1],
                            [0.707106769,-0.707106769,0]])
T_tool[:3, 3] = np.array([0, 0.371450037, 1.00605905])
tool_mesh.transform(T_tool)
tool_mesh.compute_vertex_normals()
origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

cabinet_mesh = o3d.io.read_triangle_mesh('/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/cabinet_meshes/handleless/cabinet_static_0.ply')
cabinet_mesh.compute_vertex_normals()
T_A_S = np.array([[ 0.92662295, -0.3759919 ,  0.        , -0.22462121],
       [ 0.3759919 ,  0.92662295,  0.        ,  0.59205694],
       [ 0.        ,  0.        ,  1.        ,  0.36872031],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
cabinet_mesh.transform(T_A_S)

# Create Open3D Visualization
o3d.visualization.draw_geometries([cylinder1, cylinder2, origin_rf, tool_mesh, cabinet_mesh],
                                  window_name="VTK vs. FCL Cylinders",
                                  width=800, height=600)
