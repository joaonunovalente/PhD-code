#!/usr/bin/env python

import open3d as o3d
import numpy as np

# Load source and target meshes
source_mesh = o3d.io.read_triangle_mesh("../data/domino-plane.ply")
target_mesh = o3d.io.read_triangle_mesh("../data/domino-box.ply")

# Convert both meshes to point clouds
source = source_mesh.sample_points_uniformly(number_of_points=100)
target = target_mesh.sample_points_uniformly(number_of_points=1000)

# Define the rotation angle in degrees and convert to radians
angle_degrees = 45
angle_radians = np.deg2rad(angle_degrees)  # Convert to radians

# Create a rotation matrix for a rotation around the z-axis
R = source.get_rotation_matrix_from_axis_angle([0, 0, angle_radians])

# Rotate the source point cloud
source.rotate(R, center=(0, 0, 0))
source.translate((-10, 100, 20))  # Translate the source point cloud

# Visualize the initial alignment after rotation
o3d.visualization.draw_geometries([source, target], window_name="Initial Alignment After Rotation")

# Apply ICP (Iterative Closest Point) algorithm
threshold = 200  # Distance threshold for ICP
trans_init = np.eye(4)  # Initial transformation matrix (identity matrix)
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

# Transform the source point cloud to align with the target
source.transform(reg_p2p.transformation)

# Visualize the aligned point clouds
o3d.visualization.draw_geometries([source, target], window_name="Aligned Point Clouds")

# Print the transformation matrix
print("Transformation Matrix:")
print(reg_p2p.transformation)
