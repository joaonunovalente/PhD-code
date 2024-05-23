#!/usr/bin/env python

# PhD
# Joao Nuno Valente, DEM, UA

import open3d as o3d
import numpy as np

# Viewpoint parameters for visualization
view = {
    "class_name": "ViewTrajectory",
    "interval": 29,
    "is_loop": False,
    "trajectory": [
        {
            "boundingbox_max": [17467.259999999998, 14971.680000000002, 34195.0],
            "boundingbox_min": [-10064.794, -3335.2740000000003, -120.0],
            "field_of_view": 60.0,
            "front": [0.062381731958197795, 0.1115926953029294, -0.99179412675863488],
            "lookat": [-1850.9548513560126, -1852.7030779745755, 10558.775496249387],
            "up": [-0.027821143917003417, 0.99353782152477599, 0.1100389983185585],
            "zoom": 0.3412
        }
    ],
    "version_major": 1,
    "version_minor": 0
}

def main():

    # --------------------------------------
    # Initialization
    # --------------------------------------
    filenames = [
        'data/point-clouds/0280-500.ply',
        'data/point-clouds/0380-500.ply',
        'data/point-clouds/0520-500.ply',
        'data/point-clouds/0660-500.ply',
        'data/point-clouds/0760-500.ply',
        'data/point-clouds/0890-500.ply'
    ]
    
    filename = filenames[1]  # Choose which file to load
    print('Loading file: ' + filename)
    point_cloud_original = o3d.io.read_point_cloud(filename)
    
    if point_cloud_original.is_empty():
        print("Failed to load point cloud.")
        return
    
    # --------------------------------------
    # Downsampling
    # --------------------------------------
    voxel_size = 100
    point_cloud_downsampled = point_cloud_original

    point_cloud_downsampled = point_cloud_original.voxel_down_sample(voxel_size=voxel_size)
    print("Number of points after downsampling: " + str(len(point_cloud_downsampled.points)))

    # --------------------------------------
    # Axis positioning
    # --------------------------------------
    # Create transformation T with rotation and translation
    T = np.eye(4)
    R = point_cloud_downsampled.get_rotation_matrix_from_xyz((0, 0, np.pi))
    T[0:3, 0:3] = R
    T[0:3, 3] = [0, -1400, 15000]
    
    # Apply transformation
    point_cloud_downsampled.transform(np.linalg.inv(T))
    
    # Create table reference system
    frame_world = o3d.geometry.TriangleMesh().create_coordinate_frame(size=2000, origin=np.array([0., 0., 0.]))

    # --------------------------------------
    # Bounding Box
    # --------------------------------------
    sx, sy, sz_top, sz_bottom = 3000, 5000, 3000, -2000
    np_vertices = np.array([
        [sx, sy, sz_top], [sx, -sy, sz_top], [-sx, -sy, sz_top], [-sx, sy, sz_top],
        [sx, sy, sz_bottom], [sx, -sy, sz_bottom], [-sx, -sy, sz_bottom], [-sx, sy, sz_bottom]
    ])
    
    print('np_vertices =\n' + str(np_vertices))
    
    vertices = o3d.utility.Vector3dVector(np_vertices)
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(vertices)
    print(bounding_box)
    
    point_cloud_cropped = point_cloud_downsampled.crop(bounding_box)

    point_cloud_downsampled.paint_uniform_color([0.4, 0.3, 0.3])
    # point_cloud_cropped.paint_uniform_color([0.9, 0.0, 0.0])
    print(point_cloud_cropped)

    # Save the cropped point cloud
    output_filename = 'data/point-clouds/cropped_point_cloud-0380-voxel.ply'
    o3d.io.write_point_cloud(output_filename, point_cloud_cropped)
    # --------------------------------------
    # Visualization
    # --------------------------------------
    entities = [frame_world, point_cloud_downsampled, point_cloud_cropped]
    o3d.visualization.draw_geometries(entities,
                                      zoom=view['trajectory'][0]['zoom'],
                                      front=view['trajectory'][0]['front'],
                                      lookat=view['trajectory'][0]['lookat'],
                                      up=view['trajectory'][0]['up'])

if __name__ == "__main__":
    main()
