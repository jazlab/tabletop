"""Camera calibration from checkerboard pattern detection in video.

This module provides tools for performing camera intrinsic calibration
using a checkerboard pattern video. It detects checkerboard corners
across video frames and computes the camera matrix and distortion
coefficients using OpenCV's calibration algorithms.

Functions:
    calibrate_camera: Main calibration function.
    main: CLI entry point for camera calibration.

Typical Usage:
    # Calibrate from video file
    python -m tabletop_py.gaze.calibrate_camera video.mp4 -o calib.npz

    # Load calibration data
    npz = np.load("calib.npz")
    camera_matrix = npz["camera_matrix"]
    dist_coeffs = npz["dist_coeffs"]

Dependencies:
    Requires OpenCV (cv2) and NumPy.
"""

import argparse
import os

import cv2
import numpy as np


def calibrate_camera(video_path, output_file, checkerboard_size=(9, 6)):
    """Calibrate camera intrinsics from checkerboard video frames.

    Detects checkerboard corner patterns across video frames, refines
    corner locations, and computes camera matrix and distortion
    coefficients using OpenCV's calibration algorithm.

    Args:
        video_path: Path to video file containing checkerboard views.
        output_file: Output path for .npz file containing "camera_matrix"
            and "dist_coeffs" arrays.
        checkerboard_size: (width, height) tuple of inner checkerboard
            corner counts. Default (9, 6) for 9x6 checkerboards.

    Raises:
        FileNotFoundError: If video_path doesn't exist or can't be opened.

    Notes:
        - Requires at least 10 good frames for reasonable calibration
        - Re-projection error is computed and printed
        - Saves results to .npz format (loadable with numpy.load)
    """
    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros(
        (checkerboard_size[0] * checkerboard_size[1], 3), np.float32
    )
    objp[:, :2] = np.mgrid[
        0 : checkerboard_size[0], 0 : checkerboard_size[1]
    ].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return

    print("Processing video frames...")
    frame_count = 0
    processed_frames = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret_corners, corners = cv2.findChessboardCorners(
            gray, checkerboard_size, None
        )

        # If found, add object points, image points (after refining them)
        if ret_corners:
            objpoints.append(objp)
            # Refine corner locations
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                ),
            )
            imgpoints.append(corners2)
            processed_frames += 1

            # Draw and display the corners (optional)
            # cv2.drawChessboardCorners(frame, checkerboard_size, corners2, ret_corners)
            # cv2.imshow('img', frame)
            # cv2.waitKey(1) # Adjust waitKey delay as needed

    cap.release()
    # cv2.destroyAllWindows() # if imshow was used

    if not imgpoints:
        print("Checkerboard not found in any frame. Calibration failed.")
        return

    if (
        len(imgpoints) < 10
    ):  # Need at least 10 good frames for decent calibration
        print(
            f"Warning: Only {len(imgpoints)} frames with checkerboards found. Calibration might be inaccurate."
        )

    print(
        f"Found checkerboard in {processed_frames} out of {frame_count} frames."
    )
    print("Calibrating camera...")

    if not gray.shape[::-1]:
        print("Error: Could not get frame dimensions for calibration.")
        return

    try:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        if ret:
            print("Camera calibrated successfully.")
            print("Camera Matrix:\n", camera_matrix)
            print("Distortion Coefficients:\n", dist_coeffs)

            # Save the calibration results
            np.savez(
                output_file,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
            )
            print(f"Calibration data saved to {output_file}")

            # Calculate re-projection error
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(
                    objpoints[i],
                    rvecs[i],
                    tvecs[i],
                    camera_matrix,
                    dist_coeffs,
                )
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(
                    imgpoints2
                )
                mean_error += error
            print(f"Total re-projection error: {mean_error / len(objpoints)}")

        else:
            print("Calibration failed.")
    except cv2.error as e:
        print(f"OpenCV Error during calibration: {e}")
        print(
            "This might be due to insufficient points or poor quality images."
        )
        print(f"Number of object points arrays: {len(objpoints)}")
        print(f"Number of image points arrays: {len(imgpoints)}")
        if len(imgpoints) > 0:
            print(f"Shape of first image points array: {imgpoints[0].shape}")
            print(f"Shape of first object points array: {objpoints[0].shape}")
        print(f"Image dimensions used for calibration: {gray.shape[::-1]}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calibrate camera using a checkerboard video."
    )
    parser.add_argument("video_file", help="Path to the video file.")
    parser.add_argument(
        "-o",
        "--output",
        default="camera_calibration.npz",
        help="Path to save the camera matrix and distortion coefficients (default: camera_calibration.npz)",
    )
    parser.add_argument(
        "-c",
        "--checkerboard",
        type=str,
        default="9,6",
        help="Checkerboard inner corners (width,height), e.g., '9,6'.",
    )

    args = parser.parse_args()

    try:
        checkerboard_dims = tuple(map(int, args.checkerboard.split(",")))
        if len(checkerboard_dims) != 2:
            raise ValueError(
                "Checkerboard dimensions must be two integers (width,height)."
            )
    except ValueError as e:
        print(f"Error parsing checkerboard dimensions: {e}")
        exit(1)

    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    calibrate_camera(args.video_file, args.output, checkerboard_dims)
