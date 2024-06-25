#!/usr/bin/env python

import cv2 as cv
import numpy as np
import open3d as o3d
import os

def load_images(registered_color_path, depth_path):
    registered_color_image = cv.imread(registered_color_path, cv.IMREAD_COLOR)
    depth_image = cv.imread(depth_path, cv.IMREAD_UNCHANGED).astype(np.float32) / 255.0  # Normalize back to 0-1 range
    return registered_color_image, depth_image

def create_point_cloud(registered_color_image, depth_image, intrinsic_matrix):
    h, w = depth_image.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')

    valid = (depth_image > 0)
    z = depth_image[valid]
    x = (i[valid] - intrinsic_matrix[0, 2]) * z / intrinsic_matrix[0, 0]
    y = (j[valid] - intrinsic_matrix[1, 2]) * z / intrinsic_matrix[1, 1]

    points = np.stack((x, y, z), axis=-1)
    colors = registered_color_image[valid]

    return points, colors

def save_point_cloud(points, colors, output_file):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)  # Normalize color values to 0-1
    o3d.io.write_point_cloud(output_file, pcd)

def main():
    registered_color_path = '../example_green_box/registered_image_1.png'
    depth_path = '../example_green_box/depth_frame_1.png'
    output_file = '../example_green_box/point_cloud_frame_1.ply'

    registered_color_image, depth_image = load_images(registered_color_path, depth_path)

    # Example intrinsic matrix, you should replace it with the actual intrinsic matrix of your Kinect
    intrinsic_matrix = np.array([
        [300.0, 0.0, 319.5],
        [0.0, 300.0, 239.5],
        [0.0, 0.0, 1.0]
    ])

    points, colors = create_point_cloud(registered_color_image, depth_image, intrinsic_matrix)
    save_point_cloud(points, colors, output_file)

    print(f"Saved point cloud to {output_file}")

if __name__ == "__main__":
    main()
