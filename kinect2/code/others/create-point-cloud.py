#!/usr/bin/env python

import cv2
import numpy as np
import open3d as o3d


def depth_to_point_cloud(depth_img, rgb_img, fx, fy, cx, cy):
    rows, cols = depth_img.shape
    points = []

    for y in range(rows):
        for x in range(cols):
            Z = depth_img[y, x]
            if Z == 0:  # Skip points with zero depth
                continue
            X = (x - cx) * Z / fx
            Y = (y - cy) * Z / fy
            color = rgb_img[y, x]
            points.append([X, Y, Z, *color])

    return np.array(points)

def main():
    frame = 'frame_1'
    # Load depth image and RGB image
    depth_img = cv2.imread('../../data/depth_frame_1.png', cv2.IMREAD_ANYDEPTH)
    rgb_img = cv2.imread('../../data/registered_image_1.png')

    # Intrinsic parameters (fx, fy, cx, cy)
    fx, fy, cx, cy = 320, 200, 256, 185

    # Convert depth image to point cloud
    points = depth_to_point_cloud(depth_img, rgb_img, fx, fy, cx, cy)

    # Create PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(points[:, 3:] / 255.0)  # Normalize RGB values
    print(pcd.points)

    # Save point cloud as .ply file
    o3d.io.write_point_cloud('../results/' + frame + '.ply', pcd)

if __name__ == "__main__":
    main()
