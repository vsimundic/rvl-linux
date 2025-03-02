import numpy as np
import open3d as o3d
import fcl


def load_mesh_from_ply(ply_file):
    """
    Load a mesh from a PLY file and create an FCL BVHModel for collision detection.
    
    Args:
        ply_file (str): Path to the PLY mesh file.

    Returns:
        fcl.BVHModel: Collision model of the loaded mesh.
        o3d.geometry.TriangleMesh: Open3D mesh for visualization.
    """
    mesh = o3d.io.read_triangle_mesh(ply_file)
    
    if not mesh.has_triangles():
        raise ValueError(f"The mesh in {ply_file} does not contain any triangles.")

    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    triangles = np.asarray(mesh.triangles, dtype=np.int32)

    fcl_mesh = fcl.BVHModel()
    fcl_mesh.beginModel(len(vertices), len(triangles))
    fcl_mesh.addSubModel(vertices, triangles)
    fcl_mesh.endModel()
    
    return fcl_mesh, mesh


def create_collision_object(geometry, pose):
    """
    Create a collision object for a given geometry and transformation pose.

    Args:
        geometry: FCL geometry object (e.g., BVHModel, Cylinder).
        pose (np.array): 4x4 transformation matrix.

    Returns:
        fcl.CollisionObject: Collision object with the assigned transformation.
    """
    transform = fcl.Transform(pose[:3, :3], pose[:3, 3])
    return fcl.CollisionObject(geometry, transform)


def check_collision(obj1, obj2):
    """
    Perform collision checking between two collision objects.

    Args:
        obj1, obj2 (fcl.CollisionObject): Collision objects to be checked.

    Returns:
        int: Number of contacts detected.
    """
    collision_request = fcl.CollisionRequest()
    collision_result = fcl.CollisionResult()
    
    num_contacts = fcl.collide(obj1, obj2, collision_request, collision_result)

    if num_contacts > 0:
        print(f"Collision detected! Number of contacts: {num_contacts}")
        for contact in collision_result.contacts:
            print(f"Contact point: {contact.pos}, depth: {contact.penetration_depth}")
    else:
        print("No collision detected.")

    return num_contacts


def create_o3d_cylinder(radius, height, pose):
    """
    Create an Open3D cylinder with the given radius, height, and transformation.

    Args:
        radius (float): Radius of the cylinder.
        height (float): Height of the cylinder.
        pose (np.array): 4x4 transformation matrix.

    Returns:
        o3d.geometry.TriangleMesh: Open3D mesh representation of the cylinder.
    """
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
    cylinder.compute_vertex_normals()
    cylinder.paint_uniform_color([0.8, 0.2, 0.2])  # Red color for visibility

    # Open3D creates cylinders centered at the origin; shift it upwards
    # translation = np.eye(4)
    # translation[2, 3] = height / 2  # Move to align with FCL cylinder

    # Apply transformation
    # cylinder.transform(pose @ translation)
    
    return cylinder


def visualize_scene(meshes, transformations):
    """
    Visualize the scene using Open3D.

    Args:
        meshes (list of o3d.geometry.TriangleMesh): List of Open3D meshes.
        transformations (list of np.array): List of 4x4 transformation matrices.
    """
    scene_objects = []
    for mesh, transform in zip(meshes, transformations):
        mesh_copy = mesh.transform(transform)
        scene_objects.append(mesh_copy)

    o3d.visualization.draw_geometries(scene_objects)


def main():
    # Define the cylinder (simulating the tool or object)
    radius = 0.038
    height = 0.293
    T_cylinder = np.eye(4)
    T_cylinder[:3, :3] = np.array([0.116009,0.0301308,0.992791,-0.960909,-0.249575,0.119858,0.251387,-0.967887,-4.37114e-08]).reshape((3, 3))
    T_cylinder[:3, 3] = np.array([-0.0699109,0.579074,0.368596])

    cylinder = fcl.Cylinder(radius, height)
    cylinder_object = create_collision_object(cylinder, T_cylinder)

    # Load cabinet mesh and create its collision object
    cabinet_ply_path = "/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/cabinet_meshes/handleless/cabinet_static_0.ply"
    cabinet_mesh_fcl, cabinet_mesh_o3d = load_mesh_from_ply(cabinet_ply_path)

    T_cabinet = np.array([
        [ 0.92662295, -0.3759919 ,  0.        , -0.22462121],
        [ 0.3759919 ,  0.92662295,  0.        ,  0.59205694],
        [ 0.        ,  0.        ,  1.        ,  0.36872031],
        [ 0.        ,  0.        ,  0.        ,  1.        ]
    ])
    
    cabinet_object = create_collision_object(cabinet_mesh_fcl, T_cabinet)

    # Perform collision check
    check_collision(cabinet_object, cylinder_object)

    # Create Open3D cylinder visualization
    cylinder_mesh_o3d = create_o3d_cylinder(radius, height, T_cylinder)

    # Visualize the scene with Open3D
    visualize_scene([cabinet_mesh_o3d, cylinder_mesh_o3d], [T_cabinet, T_cylinder])


if __name__ == "__main__":
    main()