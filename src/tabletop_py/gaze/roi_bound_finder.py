"""Interactive visualization tool for ROI selection using pixel coordinates.

This module displays images or video frames with grid overlay to help
identify pixel coordinates for regions of interest (ROI). The grid is
drawn every 50 pixels by default and can be used for manual ROI
boundary specification for image processing tasks.

Functions:
    show_image_with_grid: Display image/video with coordinate grid.

Typical Usage:
    # View image with grid
    show_image_with_grid("frame.jpg")

    # View random frames from video with grid (useful for LED detection ROI)
    show_image_with_grid("video.mp4", num_frames=10)

Grid spacing can be adjusted by modifying the xticks/yticks range
parameters in the function.

Dependencies:
    Requires OpenCV, Matplotlib, NumPy.
"""

import random

import cv2
import matplotlib.pyplot as plt


def show_image_with_grid(file_path, num_frames=5):
    """Display image or video frames with pixel coordinate grid overlay.

    Loads image or video, converts from OpenCV BGR to matplotlib RGB,
    and displays with grid every 50 pixels (adjustable) for visual ROI
    coordinate identification.

    Args:
        file_path: Path to image (.png, .jpg, .jpeg, .bmp, .tiff) or
            video (.avi, .mp4, .mov) file.
        num_frames: For videos, number of random frames to extract and
            display (default 5).

    Notes:
        - Grid spacing (50 pixels) can be modified by changing the
            arange() values in xticks/yticks range() calls
        - Use axis labels to identify pixel coordinates for ROI bounds
        - ROI bounds format: (x_start, x_end, y_start, y_end)
    """
    # Check if the file is an image or a video
    if file_path.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tiff")):
        # Read the image from the specified path
        img = cv2.imread(file_path)
        if img is None:
            print("Error: Image not found or unable to load.")
            return

        # Convert the color format from BGR (used by OpenCV) to RGB (used by matplotlib)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Display the image using matplotlib
        plt.imshow(img)
        plt.grid(True)  # Enable the grid

        # Set the ticks to show every 50 pixels, adjust as necessary for different image resolutions
        plt.xticks(range(0, img.shape[1], 50), rotation=90)
        plt.yticks(range(0, img.shape[0], 50))

        # Show the image with the grid overlay
        plt.show()

    elif file_path.lower().endswith((".avi", ".mp4", ".mov")):
        # Open the video file
        video = cv2.VideoCapture(file_path)
        if not video.isOpened():
            print("Error: Unable to open the video file.")
            return

        # Get the total number of frames in the video
        total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

        # Generate random frame indices
        frame_indices = random.sample(
            range(total_frames), min(num_frames, total_frames)
        )

        # Read and display the random frames
        for frame_index in frame_indices:
            video.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ret, frame = video.read()
            if ret:
                # Convert the color format from BGR (used by OpenCV) to RGB (used by matplotlib)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Display the frame using matplotlib
                plt.imshow(frame)
                plt.grid(True)  # Enable the grid

                # Set the ticks to show every 50 pixels, adjust as necessary for different video resolutions
                plt.xticks(range(0, frame.shape[1], 50), rotation=90)
                plt.yticks(range(0, frame.shape[0], 50))

                # Show the frame with the grid overlay
                plt.show()

        # Release the video file
        video.release()

    else:
        print(
            "Error: Unsupported file format. Please provide an image or video file."
        )


# Example usage:
# Replace the following path with the path to your image or video file
file_path = (
    "/Volumes/Extreme SSD/5_17_24_t2_optitrack_000-Camera 9 (513648).avi"
)
show_image_with_grid(file_path)
