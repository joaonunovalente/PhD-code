#!/usr/bin/env python

import os
import cv2
import numpy as np
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, Registration, Frame

def setup_directories(base_dir):
    """Create directories to save frames if they don't exist."""
    color_dir = os.path.join(base_dir, "color")
    depth_dir = os.path.join(base_dir, "depth")
    os.makedirs(color_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    return color_dir, depth_dir

def main():
    base_dir = "../data"
    color_dir, depth_dir = setup_directories(base_dir)

    # Initialize the library
    fn = Freenect2()
    num_devices = fn.enumerateDevices()
    if num_devices == 0:
        print("No device connected!")
        return

    # Open the device
    serial = fn.getDeviceSerialNumber(0)
    device = fn.openDevice(serial)

    # Setup listeners
    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

    # Register listeners
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)

    # Start the device
    device.start()

    # Registration object to map depth to color space
    registration = Registration(device.getIrCameraParams(), device.getColorCameraParams())

    # Frame count variable
    frame_count = 0

    try:
        while True:
            # Capture new frames from the listener
            frames = listener.waitForNewFrame()
            color = frames["color"]
            depth = frames["depth"]

            # Convert the color image to BGR format
            color_img = cv2.cvtColor(color.asarray(), cv2.COLOR_RGB2BGR)

            # Get the depth image as float32
            depth_img = depth.asarray(np.float32)

            # Prepare an empty image for the registered color frame
            registered_color = np.zeros((depth_img.shape[0], depth_img.shape[1], 3), dtype=np.uint8)

            # Create frame objects to hold the registered output
            undistorted = Frame(depth_img.shape[1], depth_img.shape[0], 4)
            registered = Frame(depth_img.shape[1], depth_img.shape[0], 4)

            # Perform the mapping
            registration.apply(color, depth, undistorted, registered)

            # Convert the registered frame to a numpy array
            registered_img = registered.asarray(np.uint8)

            # Extract the color channels from the registered image
            registered_color[:, :, 0] = registered_img[:, :, 0]  # Blue
            registered_color[:, :, 1] = registered_img[:, :, 1]  # Green
            registered_color[:, :, 2] = registered_img[:, :, 2]  # Red

            # Clip depth values to the Kinect's range (0 to 4500 mm) and convert to uint16
            depth_img = np.clip(depth_img, 0, 4500)
            depth_img = (depth_img * (65535.0 / 4500.0)).astype(np.uint16)

            # Clip RGB and depth images
            registered_color = registered_color[20:390, :]
            depth_img = depth_img[20:390, :]

            # Normalize depth image for display
            normalized_depth_img = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
            normalized_depth_img = np.uint8(normalized_depth_img)

            # Display images using OpenCV
            cv2.imshow("Color", color_img)
            cv2.imshow("Depth", normalized_depth_img)  # Normalize for display
            cv2.imshow("Registered Color", registered_color)

            # Save the color and depth images
            color_filename = os.path.join(color_dir, f"{frame_count:04d}.png")
            depth_filename = os.path.join(depth_dir, f"{frame_count:04d}.png")
            cv2.imwrite(color_filename, registered_img)
            cv2.imwrite(depth_filename, depth_img)

            frame_count += 1

            # Exit if the 'ESC' key is pressed
            if cv2.waitKey(1) == 27:
                break

            # Release frames to free memory
            listener.release(frames)

    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
    finally:
        # Clean up and close OpenCV windows
        device.stop()
        device.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
