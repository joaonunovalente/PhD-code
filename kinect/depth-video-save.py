#!/usr/bin/env python
import freenect
import cv2
import numpy as np


def get_depth():
    array, _ = freenect.sync_get_depth()
    return array.astype(np.uint8)

# Define video codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can use different codecs
fps = 30  # Adjust as needed
width, height = 640, 480  # Kinect depth resolution
out = cv2.VideoWriter('videos/depth_video.avi', fourcc, fps, (width, height), isColor=False)

# Capture depth frames and write to video
for _ in range(2000):  # Capture 300 frames (adjust as needed)
    depth_frame = get_depth()
    out.write(depth_frame)

# Release resources
out.release()