#!/usr/bin/env python


import open3d as o3d

# Load the point cloud
pcd = o3d.io.read_point_cloud("../data/point_cloud_frame_0.ply")
print(pcd)

# Downsample point cloud
pcd = pcd.voxel_down_sample(voxel_size=0.005)
print(pcd)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
