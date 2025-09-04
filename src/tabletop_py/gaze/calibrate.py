import argparse
import logging
import os
from collections.abc import Mapping
from typing import Any, cast

import yaml

from tabletop_py.gaze.preprocess import preprocess_data
from tabletop_py.gaze.train import train_and_evaluate

logger = logging.getLogger(__name__)


def main(args=None):
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s - %(message)s"
    )

    parser = argparse.ArgumentParser(
        description="Calibrate the gaze estimation model"
    )
    parser.add_argument(
        "-d",
        "--session-dir",
        type=str,
        default=os.path.join(os.environ["ROS_BAG_DIR"], "latest"),
        help="Path to bag directory",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default=os.path.join(
            os.environ["TABLETOP_DIR"], "config", "gaze_estimation.yaml"
        ),
        help="Path to the training config file",
    )
    parser.add_argument(
        "-t",
        "--start-time",
        type=float,
        default=0.0,
        help="The start time of the data to visualize in seconds, relative to the start of the session.",
    )
    parser.add_argument(
        "-m",
        "--marker-idx",
        type=int,
        default=0,
        help="The index of the marker to use.",
    )
    parser.add_argument(
        "-r",
        "--reprocess",
        action="store_true",
        help="Rerun all steps",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize the calibration data",
    )

    args = parser.parse_args(args)

    if not os.path.exists(args.session_dir):
        raise FileNotFoundError(
            f"Session directory not found at {args.session_dir}"
        )

    with open(args.config, "r") as f:
        config = cast(Mapping[str, Any], yaml.safe_load(f))

    # Convert ROS bags to CSV files
    eyelink_path = os.path.join(args.session_dir, "eyelink_sample.csv")
    markers_path = os.path.join(args.session_dir, "markers.csv")
    already_converted = os.path.exists(eyelink_path) and os.path.exists(
        markers_path
    )
    if args.reprocess or not already_converted:
        try:
            from tabletop_utils.rosbag import rosbag_session_to_dfs
        except ImportError:
            if not already_converted:
                raise ValueError(
                    "tabletop_utils.rosbag does not seem to be installed and the session "
                    "bags have not yet been converted to CSV files, you are probably "
                    "not in the docker container and should consider entering it in order "
                    "to convert the session bags to CSV files"
                )
        else:
            rosbag_session_to_dfs(
                args.session_dir,
                topics=["/eyelink/sample", "/markers"],
                verbose=True,
            )

    # Preprocess data
    path = os.path.join(args.session_dir, config["preprocess"]["filename"])
    already_preprocessed = os.path.exists(path)
    print(f"Path: {path}")
    print(f"Already preprocessed: {already_preprocessed}")

    if args.reprocess or not already_preprocessed:
        print("Preprocessing data...")
        preprocess_data(
            args.session_dir,
            config,
            marker_idx=args.marker_idx,
            start_time=args.start_time,
            visualize=args.visualize,
        )

    # Train and evaluate
    train_and_evaluate(args.session_dir, config, visualize=args.visualize)


if __name__ == "__main__":
    main()
