#!/usr/bin/env python

# PhD
# Joao Nuno Valente, DEM, UA

import os
import cv2
import numpy as np
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType

def create_directories(base_dir):
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    video_dir = os.path.join(base_dir, "videos")
    if not os.path.exists(video_dir):
        os.makedirs(video_dir)
    return video_dir

def initialize_kinect():
    fn = Freenect2()
    num_devices = fn.enumerateDevices()
    if num_devices == 0:
        raise RuntimeError("No device connected!")
    
    serial = fn.getDeviceSerialNumber(0)
    device = fn.openDevice(serial)
    return device, fn

def setup_listeners(device):
    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)
    return listener

def start_device(device):
    device.start()

def initialize_video_writers(video_dir, frame_size, fps=30):
    color_video_path = os.path.join(video_dir, "color.avi")
    depth_video_path = os.path.join(video_dir, "depth.avi")

    color_video_writer = cv2.VideoWriter(color_video_path, cv2.VideoWriter_fourcc(*'XVID'), fps, frame_size)
    depth_video_writer = cv2.VideoWriter(depth_video_path, cv2.VideoWriter_fourcc(*'XVID'), fps, frame_size, isColor=False)
    
    return color_video_writer, depth_video_writer

def process_frames(listener, color_writer, depth_writer, frame_size):
    while True:
        frames = listener.waitForNewFrame()
        color = frames["color"]
        depth = frames["depth"]

        color_img = cv2.cvtColor(color.asarray(), cv2.COLOR_RGB2BGR)
        color_img = cv2.resize(color_img, frame_size)  # Resize color frame to match depth frame size

        depth_img = depth.asarray() / 4500.0
        depth_img = np.clip(depth_img, 0, 1)
        depth_img = (depth_img * 255).astype(np.uint8)

        cv2.imshow("Color", color_img)
        cv2.imshow("Depth", depth_img)

        color_writer.write(color_img)
        depth_writer.write(depth_img)

        if cv2.waitKey(1) == 27:  # ESC key
            break

        listener.release(frames)

def main():
    try:
        example_dir = "data/"
        video_dir = create_directories(example_dir)

        device, fn = initialize_kinect()
        listener = setup_listeners(device)
        start_device(device)

        frame_size = (512, 424)  # Both color and depth frames will have this size

        color_writer, depth_writer = initialize_video_writers(video_dir, frame_size)

        process_frames(listener, color_writer, depth_writer, frame_size)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        if 'color_writer' in locals():
            color_writer.release()
        if 'depth_writer' in locals():
            depth_writer.release()
        cv2.destroyAllWindows()
        if 'device' in locals():
            device.stop()
            device.close()

if __name__ == "__main__":
    main()