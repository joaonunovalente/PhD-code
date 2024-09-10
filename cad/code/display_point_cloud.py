#!/usr/bin/env python


import open3d as o3d

# Load the PLY file
ply_file_path = "domino-box.PLY"
mesh = o3d.io.read_triangle_mesh(ply_file_path)

# Visualize the mesh
o3d.visualization.draw_geometries([mesh], window_name="3D Visualization")
