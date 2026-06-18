"""Convert ROS2 bag files to pandas DataFrames and CSV files.

This module provides utilities for extracting data from ROS2 bag files in
MCAP format and converting them to pandas DataFrames for analysis. It handles
recursive message flattening and supports filtering by topic.

The module was adapted from Open Source Robotics Foundation code and
contributions by Michal Sojka.

Typical usage:
    # Convert a single bag directory
    dfs = rosbag_to_dfs("/path/to/bag", topics=["/joint_states"])

    # Convert all bags in a session and save as CSV
    dfs = rosbag_session_to_dfs("/path/to/session", save=True)

    # Command line usage
    python -m tabletop_rig.utils.rosbag -d /path/to/session --topics /joint_states
"""


# Copyright 2020 Open Source Robotics Foundation, Inc.
# Copyright 2023, 2024 Michal Sojka <michal.sojka@cvut.cz>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import glob
import logging
import os
from collections.abc import Generator, Iterable
from typing import Any, Literal, Optional

import cv2
import numpy as np
import pandas as pd
import rosbag2_py
import tqdm
from numpy.typing import NDArray
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image

logger = logging.getLogger(__name__)

# Mapping from (src_encoding, dst_encoding) to cv2 color conversion code.
# ROS and OpenCV use opposite corner naming for Bayer patterns:
#   ROS bayer_rggb = OpenCV BayerBG
#   ROS bayer_bggr = OpenCV BayerRG
#   ROS bayer_gbrg = OpenCV BayerGR
#   ROS bayer_grbg = OpenCV BayerGB
_CV_CONVERSION_CODES: dict[tuple[str, str], int] = {
    # BGR <-> RGB
    ("bgr8", "rgb8"): cv2.COLOR_BGR2RGB,
    ("rgb8", "bgr8"): cv2.COLOR_RGB2BGR,
    ("bgra8", "rgba8"): cv2.COLOR_BGRA2RGBA,
    ("rgba8", "bgra8"): cv2.COLOR_RGBA2BGRA,
    # Alpha channel add/remove
    ("bgr8", "bgra8"): cv2.COLOR_BGR2BGRA,
    ("bgra8", "bgr8"): cv2.COLOR_BGRA2BGR,
    ("rgb8", "rgba8"): cv2.COLOR_RGB2RGBA,
    ("rgba8", "rgb8"): cv2.COLOR_RGBA2RGB,
    # To grayscale
    ("bgr8", "mono8"): cv2.COLOR_BGR2GRAY,
    ("rgb8", "mono8"): cv2.COLOR_RGB2GRAY,
    ("bgra8", "mono8"): cv2.COLOR_BGRA2GRAY,
    ("rgba8", "mono8"): cv2.COLOR_RGBA2GRAY,
    # From grayscale
    ("mono8", "bgr8"): cv2.COLOR_GRAY2BGR,
    ("mono8", "rgb8"): cv2.COLOR_GRAY2RGB,
    ("mono8", "bgra8"): cv2.COLOR_GRAY2BGRA,
    ("mono8", "rgba8"): cv2.COLOR_GRAY2RGBA,
    # 16-bit variants (same cv2 codes, bit depth handled by array dtype)
    ("bgr16", "rgb16"): cv2.COLOR_BGR2RGB,
    ("rgb16", "bgr16"): cv2.COLOR_RGB2BGR,
    ("bgr16", "mono16"): cv2.COLOR_BGR2GRAY,
    ("rgb16", "mono16"): cv2.COLOR_RGB2GRAY,
    ("mono16", "bgr16"): cv2.COLOR_GRAY2BGR,
    ("mono16", "rgb16"): cv2.COLOR_GRAY2RGB,
    # Bayer 8-bit -> BGR
    ("bayer_rggb8", "bgr8"): cv2.COLOR_BayerBG2BGR,
    ("bayer_bggr8", "bgr8"): cv2.COLOR_BayerRG2BGR,
    ("bayer_gbrg8", "bgr8"): cv2.COLOR_BayerGR2BGR,
    ("bayer_grbg8", "bgr8"): cv2.COLOR_BayerGB2BGR,
    # Bayer 8-bit -> RGB
    ("bayer_rggb8", "rgb8"): cv2.COLOR_BayerBG2RGB,
    ("bayer_bggr8", "rgb8"): cv2.COLOR_BayerRG2RGB,
    ("bayer_gbrg8", "rgb8"): cv2.COLOR_BayerGR2RGB,
    ("bayer_grbg8", "rgb8"): cv2.COLOR_BayerGB2RGB,
    # Bayer 8-bit -> grayscale
    ("bayer_rggb8", "mono8"): cv2.COLOR_BayerBG2GRAY,
    ("bayer_bggr8", "mono8"): cv2.COLOR_BayerRG2GRAY,
    ("bayer_gbrg8", "mono8"): cv2.COLOR_BayerGR2GRAY,
    ("bayer_grbg8", "mono8"): cv2.COLOR_BayerGB2GRAY,
    # Bayer 16-bit -> BGR
    ("bayer_rggb16", "bgr16"): cv2.COLOR_BayerBG2BGR,
    ("bayer_bggr16", "bgr16"): cv2.COLOR_BayerRG2BGR,
    ("bayer_gbrg16", "bgr16"): cv2.COLOR_BayerGR2BGR,
    ("bayer_grbg16", "bgr16"): cv2.COLOR_BayerGB2BGR,
    # Bayer 16-bit -> RGB
    ("bayer_rggb16", "rgb16"): cv2.COLOR_BayerBG2RGB,
    ("bayer_bggr16", "rgb16"): cv2.COLOR_BayerRG2RGB,
    ("bayer_gbrg16", "rgb16"): cv2.COLOR_BayerGR2RGB,
    ("bayer_grbg16", "rgb16"): cv2.COLOR_BayerGB2RGB,
    # Bayer 16-bit -> grayscale
    ("bayer_rggb16", "mono16"): cv2.COLOR_BayerBG2GRAY,
    ("bayer_bggr16", "mono16"): cv2.COLOR_BayerRG2GRAY,
    ("bayer_gbrg16", "mono16"): cv2.COLOR_BayerGR2GRAY,
    ("bayer_grbg16", "mono16"): cv2.COLOR_BayerGB2GRAY,
    # YUV
    ("yuv422", "bgr8"): cv2.COLOR_YUV2BGR_UYVY,
    ("yuv422", "rgb8"): cv2.COLOR_YUV2RGB_UYVY,
    ("yuv422", "mono8"): cv2.COLOR_YUV2GRAY_UYVY,
    ("yuv422_yuy2", "bgr8"): cv2.COLOR_YUV2BGR_YUY2,
    ("yuv422_yuy2", "rgb8"): cv2.COLOR_YUV2RGB_YUY2,
    ("yuv422_yuy2", "mono8"): cv2.COLOR_YUV2GRAY_YUY2,
}


def get_cv2_conversion_code(src_encoding: str, dst_encoding: str) -> int:
    """Get the OpenCV color conversion code for a pair of ROS encodings.

    Args:
        src_encoding: Source ROS image encoding (e.g. "bayer_rggb8",
            "bgr8", "mono8").
        dst_encoding: Target ROS image encoding.

    Returns:
        The cv2.COLOR_* integer constant for use with cv2.cvtColor.

    Raises:
        ValueError: If the conversion is not supported.
    """
    src = src_encoding.lower()
    dst = dst_encoding.lower()
    if src == dst:
        raise ValueError(
            f"Source and destination encodings are the same: {src}"
        )
    key = (src, dst)
    if key not in _CV_CONVERSION_CODES:
        raise ValueError(
            f"Unsupported conversion: {src_encoding!r} -> {dst_encoding!r}"
        )
    return _CV_CONVERSION_CODES[key]


# def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
#     """
#     Convert a sensor_msgs::Image message to an OpenCV :cpp:type:`cv::Mat`.
#
#     :param img_msg:   A :cpp:type:`sensor_msgs::Image` message
#     :param desired_encoding:  The encoding of the image data, one of the following strings:
#
#         * ``"passthrough"``
#         * one of the standard strings in sensor_msgs/image_encodings.h
#
#     :rtype: :cpp:type:`cv::Mat`
#     :raises CvBridgeError: when conversion is not possible.
#
#     If desired_encoding is ``"passthrough"``, then the returned image has the same encoding
#     as img_msg. Otherwise desired_encoding must be one of the standard image encodings
#
#     This function returns an OpenCV :cpp:type:`cv::Mat` message on success,
#     or raises :exc:`cv_bridge.CvBridgeError` on failure.
#
#     If the image only has one channel, the shape has size 2 (width and height)
#     """
#     dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
#     dtype = np.dtype(dtype)
#     dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")
#
#     img_buf = (
#         np.asarray(img_msg.data, dtype=dtype)
#         if isinstance(img_msg.data, list)
#         else img_msg.data
#     )
#
#     if n_channels == 1:
#         im = np.ndarray(
#             shape=(img_msg.height, int(img_msg.step / dtype.itemsize)),
#             dtype=dtype,
#             buffer=img_buf,
#         )
#         im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width])
#     else:
#         im = np.ndarray(
#             shape=(
#                 img_msg.height,
#                 int(img_msg.step / dtype.itemsize / n_channels),
#                 n_channels,
#             ),
#             dtype=dtype,
#             buffer=img_buf,
#         )
#         im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width, :])
#
#     # If the byte order is different between the message and the system.
#     if img_msg.is_bigendian == (sys.byteorder == "little"):
#         im = im.byteswap().newbyteorder()
#
#     if desired_encoding == "passthrough":
#         return im
#
#     from cv_bridge.boost.cv_bridge_boost import cvtColor2
#
#     try:
#         res = cvtColor2(im, img_msg.encoding, desired_encoding)
#     except RuntimeError as e:
#         raise CvBridgeError(e)
#
#     return res
#


def parse_compressed_image_format(fmt: str) -> tuple[str, str, str]:
    """Parse the format string from a CompressedImage message.

    The format string has the form:
    '<original_encoding>; <compression_type> <compressed_encoding>'

    Args:
        fmt: The format string from CompressedImage.format.

    Returns:
        Tuple of (original_encoding, compressed_encoding, compression_type).
    """
    original_encoding, compressed_params = fmt.split(";")
    original_encoding = original_encoding.strip()
    compressed_params = compressed_params.strip().split(" ")
    compression_type = compressed_params[0].strip()
    compressed_encoding = compressed_params[-1].strip()

    return original_encoding, compressed_encoding, compression_type


def compressed_imgmsg_to_cv2(
    msg: CompressedImage,
    dst_encoding: Optional[Literal["original"] | str] = None,
) -> tuple[NDArray, str]:
    """Decode a CompressedImage message to a numpy array.

    Decompresses the image and optionally converts to a target encoding.

    Args:
        msg: The CompressedImage message.
        dst_encoding: Target encoding for color conversion. Can be:
            - None or matches compressed_encoding: return as-is
            - "original": convert to the image's original encoding
            - Any other valid ROS encoding: convert to that encoding

    Returns:
        Tuple of (image_array, compression_type) where image_array is
        a numpy array suitable for cv2 operations.

    Raises:
        ValueError: If the requested color conversion is unsupported.
    """
    buf = np.ndarray(shape=(1, len(msg.data)), dtype=np.uint8, buffer=msg.data)
    img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)

    original_encoding, compressed_encoding, compression_type = (
        parse_compressed_image_format(msg.format)
    )

    if (
        dst_encoding is None
        or dst_encoding == compressed_encoding
        or (
            dst_encoding == "original"
            and original_encoding == compressed_encoding
        )
    ):
        return img, compression_type

    if dst_encoding == "original":
        dst_encoding = original_encoding

    code = get_cv2_conversion_code(compressed_encoding, dst_encoding)
    img = cv2.cvtColor(img, code)

    return img, compression_type


def save_image_msg(
    msg: Image | CompressedImage, save_dir: str, force: bool = False
):
    """Save a ROS image message to disk as a compressed image file.

    Decompresses compressed images and saves them as JPEG, PNG, or TIFF
    based on the message format. Skips saving if the file already exists
    (unless force=True).

    Args:
        msg: The image message (CompressedImage or Image).
        save_dir: Directory to save the image file to.
        force: If True, overwrite existing files. If False, skip if file
            with same timestamp already exists.

    Raises:
        NotImplementedError: If msg is an uncompressed Image (conversion
            not yet implemented).
        ValueError: If the message type is neither Image nor CompressedImage.
    """
    basename = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
    path = os.path.join(save_dir, f"{basename}.*")
    existing_files = glob.glob(path)
    if len(existing_files) > 0 and not force:
        assert len(existing_files) == 1
        logger.debug(f"Already saved image {existing_files[0]}, skipping")
        return

    if isinstance(msg, Image):
        raise NotImplementedError
        # img = imgmsg_to_cv2(msg)
        # file_ext = "jpg"
    elif isinstance(msg, CompressedImage):
        img, compression_type = compressed_imgmsg_to_cv2(
            msg, dst_encoding="bgr8"
        )
        match compression_type:
            case "jpeg":
                file_ext = ".jpg"
            case "png":
                file_ext = ".png"
            case "tiff":
                file_ext = ".tiff"
            case _:
                logger.warning(
                    f"Unknown CompressedImage msg format: {msg.format}. Attempting to save as jpeg."
                )
                file_ext = ".jpg"
    else:
        raise ValueError(f"Unknown image msg type: {type(msg)}")

    os.makedirs(save_dir, exist_ok=True)
    filename = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}{file_ext}"
    path = os.path.join(save_dir, filename)

    cv2.imwrite(path, img)

    logger.debug(f"Saved img to {path}")


def gen_msg_values(
    msg: Any, prefix: str = ""
) -> Generator[tuple[str, Any], None, None]:
    """Recursively flatten a ROS message into key-value pairs.

    Traverses nested ROS messages and sequences, generating column names
    with dot notation for nested fields and bracket notation for array indices.

    Args:
        msg: A ROS message, list, or primitive value to flatten.
        prefix: The current field path prefix for nested fields.

    Yields:
        Tuples of (column_name, value) for each leaf field in the message.

    Examples:
        For a PoseStamped message, generates entries like:
        - ("header.stamp.sec", 123)
        - ("pose.position.x", 1.0)
        - ("pose.orientation.w", 1.0)
    """
    if isinstance(msg, list):
        for i, val in enumerate(msg):
            yield from gen_msg_values(val, f"{prefix}[{i}]")
    elif hasattr(msg, "get_fields_and_field_types"):
        for field, type_ in msg.get_fields_and_field_types().items():
            val = getattr(msg, field)
            full_field_name = prefix + "." + field if prefix else field
            if type_.startswith("sequence<"):
                for i, aval in enumerate(val):
                    yield from gen_msg_values(aval, f"{full_field_name}[{i}]")
            else:
                yield from gen_msg_values(val, full_field_name)
    else:
        yield prefix, msg


def topic_to_basename(topic: str) -> str:
    """Convert a ROS topic name to a safe filename.

    Removes leading '/' and replaces '/' with '_'.

    Args:
        topic: The topic name (e.g., '/joint_states').

    Returns:
        A filename-safe string (e.g., 'joint_states').
    """
    return f"{topic.lstrip('/').replace('/', '_')}"


def rosbag_to_dfs(
    bag_dir: str,
    topics: Iterable[str] | None = None,
    exclude_topics: Iterable[str] | None = None,
    convert_images: bool = False,
    save_dir: Optional[str] = None,
) -> dict[str, pd.DataFrame]:
    """Convert a ROS2 bag directory to pandas DataFrames.

    Reads all messages from the specified bag and converts each topic
    to a separate DataFrame with flattened message fields as columns.
    Each row includes a `bag_time_ns` column with the message timestamp
    in nanoseconds.

    Args:
        bag_dir: Path to the bag directory containing MCAP files.
        topics: Optional whitelist of topics to include. If None, all
            topics are included.
        exclude_topics: Optional list of topics to exclude from processing.
        verbose: If True, display a progress bar during conversion.

    Returns:
        Dictionary mapping topic names to DataFrames containing all
        messages for that topic.
    """

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "cdr")
    reader.open(storage_options, converter_options)

    num_msgs = reader.get_metadata().message_count

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    topic_tables = {}

    if topics is None:
        topics = topic_types.keys()
    topics = set(topics)
    if exclude_topics is not None:
        topics = topics - set(exclude_topics)

    img_msg_warned = False

    for _ in tqdm.tqdm(range(num_msgs)):
        (topic, data, t) = reader.read_next()

        # Skip topics that are in the exclude list or not in the include list
        if topic not in topics:
            continue

        # Get the message type and deserialize the message
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)

        if isinstance(msg, (Image, CompressedImage)):
            if convert_images:
                if save_dir is not None:
                    topic_dir = os.path.join(
                        save_dir, topic_to_basename(topic)
                    )
                    save_image_msg(msg, topic_dir)
                elif not img_msg_warned:
                    logger.warning(
                        "Image messages found but save_dir is None, skipping."
                    )
                    img_msg_warned = True
        else:
            # Get table for topic or create it if it doesn't exist
            table = topic_tables.setdefault(topic, {"columns": {}, "rows": []})

            # Update the columns with new values from the message and add the
            # message to the table
            row = {"bag_time_ns": t} | dict(gen_msg_values(msg))
            table["columns"].update(row)
            table["rows"].append(row)

    assert not reader.has_next()

    dfs: dict[str, pd.DataFrame] = {}
    for topic, table in topic_tables.items():
        dfs[topic] = pd.DataFrame(
            table["rows"], columns=table["columns"].keys()
        )

    if save_dir is not None:
        for topic, df in dfs.items():
            filename = os.path.join(
                save_dir, f"{topic_to_basename(topic)}.csv"
            )
            if os.path.exists(filename):
                os.remove(filename)
            df.to_csv(filename, index=False)
            logger.info(f"Saved {filename}")

    return dfs


def rosbag_session_to_dfs(
    session_dir: str,
    topics: Iterable[str] | None = None,
    exclude_topics: Iterable[str] | None = None,
    convert_images: bool = False,
    save: bool = True,
) -> dict[str, pd.DataFrame]:
    """Convert all ROS2 bags in a session directory to DataFrames.

    Recursively finds all MCAP files in subdirectories of the session
    directory and converts them to DataFrames. Optionally saves the
    DataFrames as CSV files in the session directory.

    Args:
        session_dir: Path to the session directory containing bag subdirectories.
        topics: Optional whitelist of topics to include. If None, all
            topics are included.
        exclude_topics: Optional list of topics to exclude from processing.
        save: If True, save each DataFrame as a CSV file in the session
            directory. Topic names are converted to filenames by replacing
            '/' with '_'.

    Returns:
        Dictionary mapping topic names to DataFrames containing all
        messages for that topic across all bags in the session.

    Raises:
        FileNotFoundError: If no MCAP files are found in the session directory.
        ValueError: If the same topic appears in multiple bags (collision).
    """

    # Recursively find all .mcap files in bag_dir
    mcap_files = glob.glob(os.path.join(session_dir, "*", "*.mcap"))

    if not mcap_files:
        raise FileNotFoundError(f"No .mcap files found in {session_dir}")

    # For each .mcap file, check if any .csv files exist in the same directory
    dfs = {}
    for mcap_file in mcap_files:
        logger.info(f"Converting {mcap_file}...")

        bag_dir = os.path.dirname(mcap_file)

        new_dfs = rosbag_to_dfs(
            bag_dir=bag_dir,
            topics=topics,
            exclude_topics=exclude_topics,
            convert_images=convert_images,
            save_dir=session_dir if save else None,
        )
        collisions = dfs.keys() & new_dfs.keys()
        if collisions:
            raise ValueError(
                f"Topic collision(s) in {session_dir}: {collisions}"
            )

        dfs.update(new_dfs)

    return dfs


def main() -> None:
    """Command-line interface for converting ROS2 bags to CSV files.

    Parses command-line arguments and converts bags in the specified session
    directory (or all directories in ROS_BAG_DIR) to CSV files.

    Command-line Arguments:
        -d, --session-dir: Path to session directory. If not provided,
            converts all session directories found in ROS_BAG_DIR.
        --topics: Whitelist of topics to include.
        --exclude-topics: Topics to exclude (default: /rosout, /parameter_events).
        -f, --force: Force overwrite of existing CSV files.
    """
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert ROS2 bag files to CSV format."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "-d",
        "--session-dir",
        type=str,
        default=os.path.join(os.environ["ROS_BAG_DIR"], "latest"),
        help="Path to the session bag directory. Default: $ROS_BAG_DIR/latest",
    )
    group.add_argument(
        "-a",
        "--all-sessions",
        action="store_true",
        default=False,
        help="Convert all session directories in $ROS_BAG_DIR",
    )
    parser.add_argument("--topics", type=str, nargs="*")
    parser.add_argument(
        "--exclude-topics",
        type=str,
        nargs="*",
        # default=["/rosout", "/parameter_events"],
        default=[],
        help="Topics to exclude",
    )
    parser.add_argument(
        "--image",
        action="store_true",
        help="Convert image data",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Force overwrite existing CSV files",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Increase logging verbosity",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Open debugpy port",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s - %(message)s",
    )

    if args.debug:
        import debugpy

        print("Debug mode enabled")
        debugpy.listen(1300)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    if args.all_sessions:
        session_dirs = glob.glob(os.path.join(os.environ["ROS_BAG_DIR"], "*"))
        if not session_dirs:
            raise ValueError(
                f"No session directories found in ROS_BAG_DIR ({os.environ['ROS_BAG_DIR']})"
            )
    else:
        session_dirs = [args.session_dir]

    session_dirs = set(
        [os.path.realpath(d) for d in session_dirs if os.path.isdir(d)]
    )

    for session_dir in session_dirs:
        csv_files = glob.glob(os.path.join(session_dir, "*.csv"))
        if csv_files and not args.force:
            logger.warning(f"{session_dir} already converted, skipping...")
            continue

        try:
            rosbag_session_to_dfs(
                session_dir=session_dir,
                topics=args.topics,
                exclude_topics=args.exclude_topics,
                convert_images=args.image,
                save=True,
            )
        except Exception as e:
            logger.error(f"Error converting {session_dir}: {e}")
            continue

        print("-" * 80)


if __name__ == "__main__":
    main()
