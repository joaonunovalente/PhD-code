#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import os


def load_meshes(source_mesh_path, target_mesh_path):
    """
    Load source and target meshes from specified file paths.

    Parameters:
    - source_mesh_path (str): The file path to the source mesh.
    - target_mesh_path (str): The file path to the target mesh.

    Returns:
    - Tuple[o3d.geometry.TriangleMesh, o3d.geometry.TriangleMesh]: 
      A tuple containing the loaded source mesh and target mesh.
    """
    source_mesh = o3d.io.read_triangle_mesh(source_mesh_path)
    target_mesh = o3d.io.read_triangle_mesh(target_mesh_path)
    return source_mesh, target_mesh


def convert_to_point_clouds(source_mesh, target_mesh):
    """
    Convert meshes to point clouds and paint them with different colors.

    Parameters:
    - source_mesh (o3d.geometry.TriangleMesh): The source mesh to convert.
    - target_mesh (o3d.geometry.TriangleMesh): The target mesh to convert.

    Returns:
    - Tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud]: 
      A tuple containing the painted source and target point clouds.
    """
    source = source_mesh.sample_points_uniformly(number_of_points=10_000)
    target = target_mesh.sample_points_uniformly(number_of_points=100_000)
    
    # Paint the point clouds
    source.paint_uniform_color([1, 0.984, 0])  # Yellow color for source
    target.paint_uniform_color([0, 0.784, 1])  # Light blue color for target
    
    return source, target


def rotate_and_translate_point_cloud(source, angle_degrees, translation):
    """
    Rotate and translate the source point cloud.

    Parameters:
    - source (o3d.geometry.PointCloud): The point cloud to be transformed.
    - angle_degrees (float): The angle in degrees to rotate the point cloud.
    - translation (Tuple[float, float, float]): A tuple representing the translation (x, y, z).

    Returns:
    - o3d.geometry.PointCloud: The rotated and translated point cloud.
    """
    angle_radians = np.deg2rad(angle_degrees)
    
    # Create a rotation matrix for a rotation around the z-axis
    R = source.get_rotation_matrix_from_axis_angle([0, 0, angle_radians])
    
    # Rotate and translate the source point cloud
    source.rotate(R, center=(0, 0, 0))
    source.translate(translation)
    
    return source


def save_point_cloud(point_cloud, file_path):
    """
    Save the point cloud to a specified file path.

    Parameters:
    - point_cloud (o3d.geometry.PointCloud): The point cloud to save.
    - file_path (str): The file path where the point cloud will be saved.

    Returns:
    - None
    """
    o3d.io.write_point_cloud(file_path, point_cloud)
    print(f"Point cloud saved to {file_path}")


def apply_icp(source, target, threshold):
    """
    Apply the ICP (Iterative Closest Point) algorithm to align the source point cloud with the target.

    Parameters:
    - source (o3d.geometry.PointCloud): The source point cloud to align.
    - target (o3d.geometry.PointCloud): The target point cloud for alignment.
    - threshold (float): Distance threshold for the ICP algorithm.

    Returns:
    - np.ndarray: The transformation matrix resulting from the ICP alignment.
    """
    trans_init = np.eye(4)  # Initial transformation matrix (identity matrix)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    source.transform(reg_p2p.transformation)
    return reg_p2p.transformation


def main():
    # File paths and parameters
    source_mesh_path = "../data/domino-plane.ply"
    target_mesh_path = "../data/domino-box.ply"
    results_dir = "../results"
    angle_degrees = 45  # Rotation angle for the source point cloud
    threshold = 200      # Distance threshold for ICP

    # Ensure the output directory exists
    os.makedirs(results_dir, exist_ok=True)

    # Load source and target meshes
    source_mesh, target_mesh = load_meshes(source_mesh_path, target_mesh_path)

    # Convert both meshes to point clouds
    original_source, target = convert_to_point_clouds(source_mesh, target_mesh)

    # Visualize the original point clouds
    o3d.visualization.draw_geometries([original_source, target], window_name="Original Point Clouds")

    # Rotate and translate the source point cloud
    transformed_source = rotate_and_translate_point_cloud(original_source, angle_degrees, translation=(-10, 100, 20))

    # Save the original point cloud
    save_point_cloud(original_source, os.path.join(results_dir, "1. original_source_point_cloud.ply"))

    # Save the transformed source point cloud
    save_point_cloud(transformed_source, os.path.join(results_dir, "2. transformed_source_point_cloud.ply"))

    # Visualize the transformed point cloud
    o3d.visualization.draw_geometries([transformed_source, target], window_name="Transformed Point Cloud")

    # Apply ICP (Iterative Closest Point) algorithm
    transformation_matrix = apply_icp(transformed_source, target, threshold)

    # Visualize the aligned point clouds
    o3d.visualization.draw_geometries([transformed_source, target], window_name="Aligned Point Clouds")

    # Save the aligned point cloud (transformed source after ICP)
    save_point_cloud(transformed_source, os.path.join(results_dir, "3. aligned_source_point_cloud.ply"))

    # Print the transformation matrix
    print("Transformation Matrix:")
    print(transformation_matrix)


if __name__ == "__main__":
    main()