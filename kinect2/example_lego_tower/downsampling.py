#!/usr/bin/env python

import open3d as o3d

# Function to read and display a .ply file
def read_and_display_ply(file_path):
    # Read the point cloud from the .ply file
    point_cloud = o3d.io.read_point_cloud(file_path)
    
    # Check if the point cloud has been successfully read
    if not point_cloud.has_points():
        print("Failed to read the point cloud. Please check the file path and format.")
        return
    point_cloud = point_cloud.voxel_down_sample(voxel_size=200)
    # Display the point cloud
    o3d.visualization.draw_geometries([point_cloud], window_name='Open3D Point Cloud', width=800, height=600)
    
    # Save the point cloud to a new file if needed
    output_path = "0005_d30_vowel.ply"
    o3d.io.write_point_cloud(output_path, point_cloud)
    print(f"Point cloud has been saved to: {output_path}")

# Path to the .ply file
ply_file_path = '0005_d30.ply'

# Read and display the .ply file
read_and_display_ply(ply_file_path)

