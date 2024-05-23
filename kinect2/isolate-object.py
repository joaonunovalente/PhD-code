#!/usr/bin/env python

# PhD
# Joao Nuno Valente, DEM, UA


import open3d as o3d
import numpy as np


view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 17467.259999999998, 14971.680000000002, 34195.0 ],
			"boundingbox_min" : [ -10064.794, -3335.2740000000003, -120.0 ],
			"field_of_view" : 60.0,
			"front" : [ 0.062381731958197795, 0.1115926953029294, -0.99179412675863488 ],
			"lookat" : [ -1850.9548513560126, -1852.7030779745755, 10558.775496249387 ],
			"up" : [ -0.027821143917003417, 0.99353782152477599, 0.1100389983185585 ],
			"zoom" : 0.3412
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}



def main():

    # --------------------------------------
    # Initialization
    # --------------------------------------
    filename1 = 'data/point-clouds/0280-500.ply'
    filename2 = 'data/point-clouds/0380-500.ply'
    filename3 = 'data/point-clouds/0520-500.ply'
    filename4 = 'data/point-clouds/0660-500.ply'
    filename5 = 'data/point-clouds/0760-500.ply'
    filename6 = 'data/point-clouds/0890-500.ply'

    print('Loading file: ' + filename2)
    point_cloud_original = o3d.io.read_point_cloud(filename1)

    # --------------------------------------
    # Downsampling
    # --------------------------------------
    point_cloud_downsampled = point_cloud_original.voxel_down_sample(voxel_size=200)
    print("Number of points: " + str(point_cloud_downsampled))

    # # --------------------------------------
    # # Axis positioning
    # # --------------------------------------
    # frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=1000, origin=np.array([0., 0., 0.]))
    #
    # # Rotate and Translate the axis
    # frame.rotate(frame.get_rotation_matrix_from_xyz((0, 0, np.pi)), center=(0, 0, 0))
    # frame = frame.translate((0, -1600, 15000), relative=False)

    # --------------------------------------
    # Axis positioning
    # --------------------------------------

    # Create transformation T1 only with rotation
    T = np.zeros((4, 4), dtype=float)

    # Add homogeneous coordinates
    T[3, 3] = 1

    # Add null rotation
    R = point_cloud_downsampled.get_rotation_matrix_from_xyz((0, 0, np.pi))
    T[0:3, 0:3] = R
    # T[0:3, 0] = [1, 0, 0]   # add n vector
    # T[0:3, 1] = [0, 1, 0]   # add s vector
    # T[0:3, 2] = [0, 0, pi]  # add a vector

    # Add a translation
    T[0:3, 3] = [0, -1400, 15000]

    # Apply general transform: rotations + translation
    point_cloud_downsampled = point_cloud_downsampled.transform(np.linalg.inv(T))

    # Create table ref system and apply transformation to it
    frame_world = o3d.geometry.TriangleMesh().create_coordinate_frame(size=2000, origin=np.array([0., 0., 0.]))

    # --------------------------------------
    # Bounding Box
    # --------------------------------------
    np_vertices = np.ndarray([8, 3])

    sx = sy = 5000
    sz_top = 0
    sz_bottom = -5000
    np_vertices[0, 0:3] = [sx, sy, sz_top]
    np_vertices[1, 0:3] = [sx, -sy, sz_top]
    np_vertices[2, 0:3] = [-sx, -sy, sz_top]
    np_vertices[3, 0:3] = [-sx, sy, sz_top]
    np_vertices[4, 0:3] = [sx, sy, sz_bottom]
    np_vertices[5, 0:3] = [sx, -sy, sz_bottom]
    np_vertices[6, 0:3] = [-sx, -sy, sz_bottom]
    np_vertices[7, 0:3] = [-sx, sy, sz_bottom]

    print('np_vertices =\n' + str(np_vertices))

    vertices = o3d.utility.Vector3dVector(np_vertices)

    # Create a bounding box
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(vertices)
    print(bounding_box)

    point_cloud_cropped = point_cloud_downsampled.crop(bounding_box)

    point_cloud_downsampled.paint_uniform_color([0.4, 0.3, 0.3])
    point_cloud_cropped.paint_uniform_color([0.9, 0.0, 0.0])


    # --------------------------------------
    # Visualization
    # --------------------------------------
    entities = []
    # entities.append(frame)
    entities.extend([frame_world, point_cloud_downsampled, point_cloud_cropped])
    o3d.visualization.draw_geometries(entities,
                                      zoom=0.3412,
                                      front=view['trajectory'][0]['front'],
                                      lookat=view['trajectory'][0]['lookat'],
                                      up=view['trajectory'][0]['up'])


if __name__ == "__main__":
    main()