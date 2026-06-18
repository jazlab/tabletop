"""OptiTrack motion capture data processing and LED status extraction.

This module processes OptiTrack marker data and video to extract LED
(light-emitting diode) synchronization signals and merge with motion
capture positions. It uses brightness analysis on a region of interest
(ROI) to detect LED on/off states and correlate with marker positions.

Functions:
    is_led_on: Detect LED state from ROI brightness.
    extract_frame_number: Parse frame number from filename.
    process_image: Extract LED status and frame info from single image.
    process_images: Parallel image processing for LED detection.
    process_data: Full pipeline merging marker data with LED status.

Constants:
    BRIGHTNESS_THRESHOLD: Default brightness level for LED on/off (0-255).

Example:
    # Extract LED states and merge with marker data
    roi = (1250, 1275, 980, 1000)  # (x_start, x_end, y_start, y_end)
    data = process_data(
        "markers.csv", "output_merged.csv", "output_sep.csv",
        "video_frames/", roi, merge_markers=True
    )

Dependencies:
    Requires OpenCV, NumPy, Pandas.
"""

import os
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import pandas as pd

#: Brightness threshold (0-255) for LED on/off detection
BRIGHTNESS_THRESHOLD = 150


def is_led_on(image, roi_bounds, threshold=BRIGHTNESS_THRESHOLD):
    """
    Check if the LED is on in the specified region of interest (ROI) of the image.

    Args:
        image: The input image as a numpy array.
        roi_bounds: A tuple of (x_start, x_end, y_start, y_end) specifying the ROI bounds.
        threshold: The brightness threshold to determine if the LED is on.

    Returns:
        True if the average brightness in the ROI is above the threshold, False otherwise.
    """
    x_start, x_end, y_start, y_end = roi_bounds
    roi = image[y_start:y_end, x_start:x_end]
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    average_brightness = np.mean(gray_roi)
    return average_brightness > threshold


def extract_frame_number(filename):
    """
    Extract the frame number from the filename.

    Args:
        filename: The filename of the image.

    Returns:
        The frame number as an integer.
    """
    start = filename.find("frame") + 5
    end = filename.find(".jpg")
    return int(filename[start:end])


def process_image(img_path, roi_bounds):
    """
    Process a single image to determine if the LED is on and extract the frame number.

    Args:
        img_path: The path to the image file.
        roi_bounds: A tuple of (x_start, x_end, y_start, y_end) specifying the ROI bounds.

    Returns:
        A tuple of (frame_number, led_status), where led_status is 1 if the LED is on, 0 otherwise.
    """
    img = cv2.imread(img_path)
    if img is None:
        print(f"Failed to load image: {img_path}")
        return None
    led_status = is_led_on(img, roi_bounds)
    frame_number = extract_frame_number(os.path.basename(img_path))
    return frame_number, 1 if led_status else 0


def process_images(folder_path, roi_bounds, num_threads=4):
    """
    Process all the images in a folder to determine LED status and frame numbers.

    Args:
        folder_path: The path to the folder containing the images.
        roi_bounds: A tuple of (x_start, x_end, y_start, y_end) specifying the ROI bounds.
        num_threads: The number of threads to use for parallel processing.

    Returns:
        A list of tuples, where each tuple contains (frame_number, led_status).
    """
    files = [
        f
        for f in os.listdir(folder_path)
        if f.endswith(".jpg") and not f.startswith("._")
    ]
    files = sorted(files, key=extract_frame_number)

    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        results = executor.map(
            lambda f: process_image(os.path.join(folder_path, f), roi_bounds),
            files,
        )

    frame_data = [result for result in results if result is not None]
    return frame_data


def process_data(
    marker_input_file_path,
    output_path_merged,
    output_path_separate,
    image_folder_path,
    roi_bounds,
    merge_markers=False,
):
    """Merge OptiTrack marker data with LED detection from video frames.

    Reads OptiTrack CSV (6-line header skip), extracts LED on/off states
    from video ROI brightness analysis, and outputs marker positions with
    synchronized LED status column.

    Args:
        marker_input_file_path: OptiTrack CSV with 6-line header, columns
            "Time (Seconds)" and marker position columns.
        output_path_merged: Output CSV path if merge_markers=True.
        output_path_separate: Output CSV path if merge_markers=False.
        image_folder_path: Directory containing .jpg frames with names
            like "frame00000.jpg".
        roi_bounds: (x_start, x_end, y_start, y_end) tuple defining
            grayscale brightness region.
        merge_markers: If True, combines all markers into single X/Y/Z
            columns; if False, keeps separate marker columns.

    Returns:
        DataFrame with columns:
        - "time": Seconds (3 decimal precision)
        - LED marker positions (1_X, 1_Y, 1_Z or X, Y, Z if merged)
        - "input": LED state (1=on, 0=off) mapped from frame brightness
    """
    # Read the CSV data from the file
    data = pd.read_csv(marker_input_file_path, skiprows=6)

    # Drop the first unnamed column
    data = data.drop(data.columns[0], axis=1)

    # Rename the 'Time (Seconds)' column to 'Time'
    data = data.rename(columns={"Time (Seconds)": "time"})

    # Create a new DataFrame with the desired columns
    columns = ["time"]
    marker_count = (len(data.columns) - 1) // 3
    for i in range(1, marker_count + 1):
        columns.extend([f"{i}_X", f"{i}_Y", f"{i}_Z"])

    new_data = pd.DataFrame(columns=columns)

    # Copy the 'Time' column and round it to three decimal places
    new_data["time"] = data["time"].round(3)

    # Iterate over the markers and copy the X, Y, Z columns
    for i in range(1, marker_count + 1):
        x_col = data.columns[3 * i - 2]
        y_col = data.columns[3 * i - 1]
        z_col = data.columns[3 * i]

        new_data[f"{i}_X"] = data[x_col]
        new_data[f"{i}_Y"] = data[y_col]
        new_data[f"{i}_Z"] = data[z_col]

    # Process the images to extract LED data
    frame_data = process_images(image_folder_path, roi_bounds)

    # Create a dictionary to store the LED status for each timestamp
    led_dict = {
        round(frame_number * 0.008333333333, 3): led_status
        for frame_number, led_status in frame_data
    }

    # Add the 'input' column to the DataFrame
    new_data["input"] = new_data["time"].map(led_dict).fillna(0).astype(int)

    if merge_markers:
        # Merge the X, Y, Z data into three columns
        merged_data = pd.DataFrame(columns=["time", "X", "Y", "Z", "input"])
        merged_data["time"] = new_data["time"]
        merged_data["input"] = new_data["input"]

        for i in range(marker_count, 0, -1):
            x_col = f"{i}_X"
            y_col = f"{i}_Y"
            z_col = f"{i}_Z"

            merged_data["X"] = new_data[x_col].combine_first(merged_data["X"])
            merged_data["Y"] = new_data[y_col].combine_first(merged_data["Y"])
            merged_data["Z"] = new_data[z_col].combine_first(merged_data["Z"])

        # Save the merged data to a CSV file
        merged_data.to_csv(output_path_merged, index=False)
        return merged_data
    else:
        # Save the separate marker data to a CSV file
        new_data.to_csv(output_path_separate, index=False)
        return new_data


# Example usage
image_folder_path = (
    "/Volumes/Extreme SSD/5_21_24_t1/5_21_24_t1_optitrack-513648"
)
marker_input_file_path = (
    "/Volumes/Extreme SSD/5_21_24_t1/Raw/5_21_24_t1_optitrack.csv"
)
output_file_path_merged = (
    "/Users/jack/Downloads/5_21_24_t1/Raw/5_21_24_t1_optitrack.csv"
)
output_file_path_separate = (
    "/Users/jack/Downloads/5_15_t3/Processed/5_15_t3_marker_seperated.csv"
)
roi_bounds = (
    1250,
    1275,
    980,
    1000,
)  # Update these values based on your ROI (x_start, x_end, y_start, y_end)

result_merged = process_data(
    marker_input_file_path,
    output_file_path_merged,
    output_file_path_separate,
    image_folder_path,
    roi_bounds,
    merge_markers=True,
)
print("Merged Data:")
print(result_merged)

# Uncomment this section if you have multiple markers that you are tracking in the data
"""
result_separate = process_data(marker_input_file_path, output_file_path_merged, output_file_path_separate, image_folder_path, roi_bounds, merge_markers=False)
print("\nSeparate Marker Data:")
print(result_separate)
"""
