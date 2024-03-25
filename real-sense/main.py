import pyrealsense2 as rs

# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Create a configuration object
config = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for the next set of frames
        frames = pipeline.wait_for_frames()

        # Get the color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Process the frames here...
        # For example, you can convert them to numpy arrays and display them

finally:
    # Stop the pipeline
    pipeline.stop()
