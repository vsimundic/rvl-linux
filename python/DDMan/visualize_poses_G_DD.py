import numpy as np
import open3d as o3d
import fcl

def load_fcl_mesh_from_ply(ply_file):
    """
    Load a mesh from a PLY file and create an FCL BVHModel.
    """
    # Load mesh using Open3D
    mesh = o3d.io.read_triangle_mesh(ply_file)
    if not mesh.has_triangles():
        raise ValueError(f"The mesh in {ply_file} does not contain any triangles.")

    # Extract vertices and triangles
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    triangles = np.asarray(mesh.triangles, dtype=np.int32)

    # Create FCL BVHModel
    fcl_mesh = fcl.BVHModel()
    fcl_mesh.beginModel(len(vertices), len(triangles))
    fcl_mesh.addSubModel(vertices, triangles)
    fcl_mesh.endModel()
    
    return fcl_mesh


def create_collision_object(mesh, pose):
    """
    Create a collision object for a mesh with a given pose.
    """
    # Convert pose to FCL-compatible format
    transform = fcl.Transform(pose[:3, :3], pose[:3, 3])
    print("Transform rotation:\n", transform.getRotation())
    print("Transform translation:\n", transform.getTranslation())
    return fcl.CollisionObject(mesh, transform)


def visualize_scene(cabinet_file, tool_file, cabinet_pose, tool_pose):
    """
    Visualize the cabinet and tool in Open3D.
    """
    cabinet_mesh = o3d.io.read_triangle_mesh(cabinet_file)
    tool_mesh = o3d.io.read_triangle_mesh(tool_file)

    tool_mesh.transform(tool_pose)
    cabinet_mesh.transform(cabinet_pose)
    tool_mesh.compute_vertex_normals()
    cabinet_mesh.compute_vertex_normals()
    # Visualize both meshes
    o3d.visualization.draw_geometries([cabinet_mesh, tool_mesh], window_name="Collision Scene")

if __name__ == "__main__":
    cabinet_mesh_path= '/home/RVLuser/rvl-linux/python/DDMan/dd_plate_mesh.ply'
    tool_mesh_path = '/home/RVLuser/rvl-linux/data/Robotiq3Finger/mesh.ply'
    # cabinet_mesh = load_mesh_from_ply(cabinet_mesh_path)
    cabinet_mesh = o3d.io.read_triangle_mesh(cabinet_mesh_path)
    cabinet_mesh.compute_vertex_normals()
    original_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

    T_plate_0 = np.eye(4)
    fcl_cabinet_mesh = load_fcl_mesh_from_ply(cabinet_mesh_path)
    cabinet_col_obj = create_collision_object(fcl_cabinet_mesh, T_plate_0)

    with open('/home/RVLuser/rvl-linux/poses_G_DD.txt', 'r') as f:
        for line in f.readlines():
            transform_list = line.split(' ')[:12]
            transform_list = [float(i) for i in transform_list]

            rotation = np.array(transform_list[0:9]).reshape((3,3))
            translation = np.array(transform_list[9:12])
            T_G_DD = np.eye(4)
            T_G_DD[:3,:3] = rotation
            T_G_DD[:3,3] = translation

            tool_mesh = o3d.io.read_triangle_mesh(tool_mesh_path)
            tool_mesh.compute_vertex_normals()

            tool_mesh.transform(T_G_DD)
            o3d.visualization.draw_geometries([cabinet_mesh, tool_mesh, original_rf], window_name="Collision Scene")

            fcl_tool_mesh = load_fcl_mesh_from_ply(tool_mesh_path)
            tool_col_obj = create_collision_object(fcl_tool_mesh, T_G_DD)
            
            # Perform collision check
            collision_request = fcl.CollisionRequest()
            collision_result = fcl.CollisionResult()
            
            ret = fcl.collide(cabinet_col_obj, tool_col_obj, collision_request, collision_result)
            print("Is colliding: " + str(ret))
