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
import os
from collections.abc import Generator, Iterable
from typing import Any

import pandas as pd
import rosbag2_py  # noqa
import tqdm
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _gen_msg_values(
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
            yield from _gen_msg_values(val, f"{prefix}[{i}]")
    elif hasattr(msg, "get_fields_and_field_types"):
        for field, type_ in msg.get_fields_and_field_types().items():
            val = getattr(msg, field)
            full_field_name = prefix + "." + field if prefix else field
            if type_.startswith("sequence<"):
                for i, aval in enumerate(val):
                    yield from _gen_msg_values(aval, f"{full_field_name}[{i}]")
            else:
                yield from _gen_msg_values(val, full_field_name)
    else:
        yield prefix, msg


def rosbag_to_dfs(
    bag_dir: str,
    topics: Iterable[str] | None = None,
    exclude_topics: Iterable[str] | None = None,
    verbose: bool = False,
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

    for _ in tqdm.tqdm(range(num_msgs), disable=not verbose):
        (topic, data, t) = reader.read_next()

        # Skip topics that are in the exclude list or not in the include list
        if topic not in topics:
            continue

        # Get the message type and deserialize the message
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)

        # Get table for topic or create it if it doesn't exist
        table = topic_tables.setdefault(topic, {"columns": {}, "rows": []})

        # Update the columns with new values from the message and add the
        # message to the table
        row = {"bag_time_ns": t} | dict(_gen_msg_values(msg))
        table["columns"].update(row)
        table["rows"].append(row)

    assert not reader.has_next()

    dfs: dict[str, pd.DataFrame] = {}
    for topic, table in topic_tables.items():
        dfs[topic] = pd.DataFrame(
            table["rows"], columns=table["columns"].keys()
        )

    return dfs


def rosbag_session_to_dfs(
    session_dir: str,
    topics: Iterable[str] | None = None,
    exclude_topics: Iterable[str] | None = None,
    save: bool = True,
    verbose: bool = False,
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
        verbose: If True, display progress information during conversion.

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
        print(f"Converting {mcap_file}...")

        mcap_dir = os.path.dirname(mcap_file)

        new_dfs = rosbag_to_dfs(
            bag_dir=mcap_dir,
            topics=topics,
            exclude_topics=exclude_topics,
            verbose=verbose,
        )
        collisions = dfs.keys() & new_dfs.keys()
        if collisions:
            raise ValueError(
                f"Topic collision(s) in {session_dir}: {collisions}"
            )

        dfs.update(new_dfs)

    if save:
        for topic, df in dfs.items():
            filename = os.path.join(
                session_dir, f"{topic.lstrip('/').replace('/', '_')}.csv"
            )
            if os.path.exists(filename):
                os.remove(filename)
            df.to_csv(filename, index=False)
            if verbose:
                print(f"Saved {filename}")

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
    parser.add_argument(
        "-d",
        "--session-dir",
        type=str,
        help="The path to the session directory. If not provided, all session directories in ROS_BAG_DIR will be converted.",
    )
    parser.add_argument("--topics", type=str, nargs="*")
    parser.add_argument(
        "--exclude-topics",
        type=str,
        nargs="*",
        default=["/rosout", "/parameter_events"],
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Force overwrite existing CSV files.",
    )
    args = parser.parse_args()

    if args.session_dir is None:
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
            print(f"{session_dir} already converted, skipping...")
            continue

        try:
            rosbag_session_to_dfs(
                session_dir=session_dir,
                topics=args.topics,
                exclude_topics=args.exclude_topics,
                save=True,
                verbose=True,
            )
        except Exception as e:
            print(f"Error converting {session_dir}: {e}")
            continue

        print("-" * 80)


if __name__ == "__main__":
    main()
