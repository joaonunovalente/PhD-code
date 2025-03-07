#!/usr/bin/env python3

import pickle
import numpy as np
import open3d as o3d

def extract_transformation(matrix):
    translation = np.asarray(matrix[:3, 3])  # Ensure numpy array
    R = np.asarray(matrix[:3, :3])  # Convert rotation matrix to numpy array
    
    # Compute Euler angles (ZYX convention: yaw-pitch-roll)
    yaw = np.arctan2(R[1, 0], R[0, 0]).item()
    pitch = np.arcsin(-R[2, 0]).item()
    roll = np.arctan2(R[2, 1], R[2, 2]).item()
    
    # Convert to degrees
    yaw_deg = float(np.degrees(yaw))
    pitch_deg = float(np.degrees(pitch))
    roll_deg = float(np.degrees(roll))
    
    return translation.tolist(), (roll_deg, pitch_deg, yaw_deg)

def load_transformations(pickle_path):
    with open(pickle_path, 'rb') as f:
        data = pickle.load(f)
    
    est_matrix = np.asarray(data.get("estimated-transformation"))
    ref_matrix = np.asarray(data.get("refined-transformation"))
    
    est_translation, est_angles = extract_transformation(est_matrix)
    ref_translation, ref_angles = extract_transformation(ref_matrix)
    
    return est_matrix, est_translation, est_angles, ref_matrix, ref_translation, ref_angles

def apply_transformation(source_ply, transformation_matrix):
    source = o3d.io.read_point_cloud(source_ply)
    source.transform(transformation_matrix)
    return source

def downsample_point_cloud(point_cloud, voxel_size=100):
    return point_cloud.voxel_down_sample(voxel_size)

def visualize_point_clouds(source_ply, transformed_ply, target_ply=None, voxel_size=0.02):
    source = o3d.io.read_point_cloud(source_ply)
    transformed = o3d.io.read_point_cloud(transformed_ply)
    
    source = downsample_point_cloud(source, voxel_size)
    transformed = downsample_point_cloud(transformed, voxel_size)
    
    transformed.paint_uniform_color([1, 0, 0])  # Red
    point_clouds = [source, transformed]
    
    if target_ply:
        target = o3d.io.read_point_cloud(target_ply)
        target = downsample_point_cloud(target, voxel_size)
        point_clouds.append(target)
    
    o3d.visualization.draw_geometries(point_clouds)

def main():
    pickle_path = 'results/ORBBEC.pickle'
    source_ply = "dataset/ORBBEC/Front/RGBD/pointcloud.ply"
    target_ply = "dataset/ORBBEC/Side/RGBD/pointcloud.ply"
    transformed_ply = "transformed.ply"
    
    est_matrix, est_translation, est_angles, ref_matrix, ref_translation, ref_angles = load_transformations(pickle_path)
    
    print("--------- Estimated Transformation -----------")
    print(f"Translation: {est_translation}")
    print(f"Rotation (Roll, Pitch, Yaw): ({est_angles[0]:.10f}, {est_angles[1]:.10f}, {est_angles[2]:.10f})\n")
    
    print("--------- Refined Transformation -----------")
    print(f"Translation: {ref_translation}")
    print(f"Rotation (Roll, Pitch, Yaw): ({ref_angles[0]:.10f}, {ref_angles[1]:.10f}, {ref_angles[2]:.10f})")
    
    transformed = apply_transformation(source_ply, ref_matrix)
    # o3d.io.write_point_cloud(transformed_ply, transformed)
    visualize_point_clouds(source_ply, transformed_ply, target_ply, voxel_size=0.02)

if __name__ == "__main__":
    main()