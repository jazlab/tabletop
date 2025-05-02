import random

import cv2
import matplotlib.pyplot as plt


def show_image_with_grid(file_path, num_frames=5):
    """
    Display an image or random frames from a video with a grid overlay for easier visual analysis of pixel coordinates.

    This function reads an image or video from a specified path. If it's an image, it converts it from BGR to RGB format
    for correct color display and shows the image with a grid overlay. If it's a video, it extracts a specified number
    of random frames from the video and displays each frame with a grid overlay.

    The grid helps in visually determining the pixel coordinates within the image or video frames, which can be crucial
    for tasks like ROI selection.

    Args:
        file_path (str): The path to the image or video file to be displayed.
        num_frames (int, optional): The number of random frames to extract from the video. Default is 5.

    Returns:
        None: The function displays the image or video frames with matplotlib but does not return any value.
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
