# This script captures one frame from the depth camera each time when '5' is pressed on keyboard

# copy this code into the location where you want to save your pictures
# run "python capture_2d_image.py"
# and press '5' if you want to take pictures


import pyrealsense2 as rs
import cv2
import numpy as np
import os
import time
import keyboard

def get_unique_filename(base_filename):
    """
    Generate a unique filename by appending a number to the base filename
    if a file with that name already exists in the current working directory.
    """
    filename, ext = os.path.splitext(base_filename)
    index = 0
    while os.path.exists(f"{filename}({index}){ext}"):
        index += 1
    return f"{filename}({index}){ext}"

def main():
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the pipeline
    pipeline.start(config)

    capturing = False  # Flag to indicate if capturing is in progress

    try:
        while True:
            # Check if the number 5 key is pressed
            if keyboard.is_pressed('5'):
                if not capturing:
                    # Wait for a frame
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()

                    if color_frame:
                        # Convert the color frame to a numpy array
                        color_image = np.asanyarray(color_frame.get_data())

                        # Generate a unique filename
                        unique_filename = get_unique_filename("new_2d_image.jpg")

                        # Save the 2D image with the unique filename in the current working directory
                        cv2.imwrite(unique_filename, color_image)

                        print("Image captured and saved!")
                        capturing = True
                else:
                    # Wait until the key is released before capturing again
                    while keyboard.is_pressed('5'):
                        pass
                    capturing = False

    except KeyboardInterrupt:
        # Handle the KeyboardInterrupt (Ctrl+C) to gracefully exit the script
        pass
    finally:
        # Stop the pipeline when done
        pipeline.stop()

if __name__ == "__main__":
    main()
