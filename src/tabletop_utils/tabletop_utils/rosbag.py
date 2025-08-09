"""Convert ROS 2 bag to CSV files, one CSV file per topic."""


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

import os

import pandas as pd
import rosbag2_py  # noqa
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _gen_msg_values(msg, prefix=""):
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


def rosbag_to_csv(bag_dir, topics=None, exclude_topics=None):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "cdr")
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {
        topic_types[i].name: topic_types[i].type
        for i in range(len(topic_types))
    }

    table_map = {}

    if topics is None:
        topics = set(type_map.keys())
    else:
        topics = set(topics)
    if exclude_topics is not None:
        topics = topics - set(exclude_topics)

    # start_time = None
    i = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()

        # Skip topics that are in the exclude list or not in the include list
        if topic not in topics:
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic not in table_map:
            table_map[topic] = {
                "columns": [
                    "time",
                    *[field for field, _ in _gen_msg_values(msg)],
                ],
                "msgs": [],
            }

        # if hasattr(msg, "header"):
        #     t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        # else:
        # if start_time is None:
        #     start_time = ts
        row = [t, *[val for _, val in _gen_msg_values(msg)]]
        if len(row) != len(table_map[topic]["columns"]):
            print(
                f"Row length mismatch for topic {topic}: {len(row)} != {len(table_map[topic]['columns'])}"
            )
            print(f"Skipping conversion for topic {topic}")
            topics.remove(topic)
            table_map.pop(topic)
            continue
        table_map[topic]["msgs"].append(row)
        # print(
        #     ",".join(
        #         [str(ts)] + [str(val) for _, val in _gen_msg_values(msg)]
        #     ),
        #     file=file,
        # )
        if i % 1000 == 0:
            print(f"Msg: {i}, Time: {t}")
        i += 1

    for topic, topic_data in table_map.items():
        df = pd.DataFrame(topic_data["msgs"], columns=topic_data["columns"])
        filename = f"{bag_dir}/{topic.lstrip('/').replace('/', '_')}.csv"
        if os.path.exists(filename):
            print(f"Overwriting {filename}")
            os.remove(filename)
        df.to_csv(filename, index=False)
        print(f"Saved {filename}")


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d", "--dir", type=str, default=os.environ["TABLETOP_BAG_DIR"]
    )
    parser.add_argument("--topics", type=str, nargs="*")
    parser.add_argument(
        "--exclude-topics",
        type=str,
        nargs="*",
        default=["/rosout", "/parameter_events"],
    )
    args = parser.parse_args()

    import glob

    # Recursively find all .mcap files in bag_dir
    mcap_files = glob.glob(
        os.path.join(args.dir, "**", "*.mcap"), recursive=True
    )

    # For each .mcap file, check if any .csv files exist in the same directory
    for mcap_file in mcap_files:
        mcap_dir = os.path.dirname(mcap_file)
        csv_files = glob.glob(os.path.join(mcap_dir, "*.csv"))
        if not csv_files:
            print(
                f"Found .mcap file with no .csv in {mcap_dir}, "
                "converting to CSV..."
            )
            # Call rosbag_to_csv for this directory
            rosbag_to_csv(
                bag_dir=mcap_dir,
                topics=args.topics,
                exclude_topics=args.exclude_topics,
            )


if __name__ == "__main__":
    main()
