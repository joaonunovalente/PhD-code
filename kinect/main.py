#!/usr/bin/env python
import freenect
import cv2
import frame_convert2

def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])

def get_video():
    return frame_convert2.video_cv(freenect.sync_get_video()[0])

# Define the codec and create VideoWriter object
video_fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_out = cv2.VideoWriter('videos/output_video.avi', video_fourcc, 20.0, (640, 480))

# Define the codec for depth frames and create VideoWriter object
depth_fourcc = cv2.VideoWriter_fourcc(*'XVID')
depth_out = cv2.VideoWriter('videos/output_depth.avi', depth_fourcc, 20.0, (640, 480))

while True:
    depth_frame = get_depth()
    video_frame = get_video()

    # Display frames
    cv2.imshow('Depth', depth_frame)
    cv2.imshow('Video', video_frame)

    # Write frames to files
    video_out.write(video_frame)
    depth_out.write(depth_frame)

    # Break the loop if 'Esc' key is pressed
    if cv2.waitKey(10) == 27:
        break

# Release VideoWriters and close OpenCV windows
video_out.release()
depth_out.release()
cv2.destroyAllWindows()
