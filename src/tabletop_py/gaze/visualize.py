import logging
import os
from collections.abc import Mapping
from typing import Any, Optional, cast

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from tabletop_py.gaze.preprocess import format_columns, reindex_steady_time

logger = logging.getLogger(__name__)


def plot_eyelink_markers(
    df: pd.DataFrame,
    *,
    freq: float,
    markers_df: Optional[pd.DataFrame] = None,
    markers_freq: Optional[float] = None,
    title: str = "Eyelink and Marker Data",
    save_path: str | None = None,
):
    """
    Plot the eyelink and markers data.

    Args:
        df: The dataframe containing the eyelink data (and optionally markers data).
        markers_df: Optional dataframe containing the markers data if not provided in df.
        title: The title of the plot.
        save_path: The path to save the plot.
    """
    eyelink_df = reindex_steady_time(df, freq, on="time")
    # eyelink_df = df
    if markers_df is not None:
        if markers_freq is None:
            raise ValueError(
                "markers_freq must be provided if markers_df is provided"
            )
        markers_df = reindex_steady_time(markers_df, markers_freq, on="time")
        # markers_df = markers_df
    else:
        markers_df = eyelink_df

    fig, ax = plt.subplots(7, 1, sharex=True, figsize=(10, 15))
    fig.suptitle(title)
    i = 0
    for col in ["left_x", "left_y", "right_x", "right_y"]:
        ax[i].plot(eyelink_df["time"], eyelink_df[col])
        ax[i].set_title(f"Eyelink {col}")
        i += 1

    for col in ["marker_x", "marker_y", "marker_z"]:
        ax[i].plot(markers_df["time"], markers_df[col])
        ax[i].set_title(f"Markers {col}")
        i += 1

    if save_path is not None:
        plt.savefig(save_path)
    else:
        plt.show()


def animate_2d_dots(
    data: Mapping[str, np.ndarray],
    *,
    freq: float,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    fr: int = 10,
    save_path: str | None = None,
):
    """
    Animates 2D dots.

    Args:
        data: The data to animate. Should be a dictionary of arrays, where each
            key is a dot name and the value is an array of size (T, 2) for each
            time step.
        freq: The frequency of the data.
        min_x: The minimum x value.
        max_x: The maximum x value.
        min_y: The minimum y value.
        max_y: The maximum y value.
        fr: The frame rate.
        save_path: The path to save the animation.
    """
    num_samples = next(iter(data.values())).shape[0]
    interval = 1 / fr

    fig = plt.figure()
    ax = fig.add_subplot()

    plots = {}
    for key, value in data.items():
        if value.shape != (num_samples, 2):
            raise ValueError(
                f"Dot {key} has {value.shape} samples, but expected {(num_samples, 2)}"
            )
        plot = ax.scatter([], [])
        plot.set_label(key)
        plots[key] = plot

    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()

    def init():
        for key, plot in plots.items():
            plot.set_offsets(data[key][0])
        return tuple(plots.values())

    def animate(i):
        idx = int(i * interval * freq)
        for key, plot in plots.items():
            plot.set_offsets(data[key][idx])
        return tuple(plots.values())

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=int(num_samples / (interval * freq)),
        interval=interval * 1000,
        blit=True,
    )

    if save_path is None:
        plt.show()
    else:
        if save_path.endswith(".gif"):
            writer = animation.PillowWriter(fps=fr)
        elif save_path.endswith(".mp4"):
            writer = animation.FFMpegWriter(fps=fr)
        else:
            raise ValueError(f"Invalid file extension: {save_path}")

        anim.save(save_path, writer=writer)


def animate_3d_dots(
    data: Mapping[str, np.ndarray],
    *,
    freq: float,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    min_z: float,
    max_z: float,
    fr: int = 10,
    save_path: str | None = None,
):
    """
    Animates 3D dots.

    Args:
        data: The data to animate. Should be a dictionary of arrays, where each
            key is a dot name and the value is an array of size (T, 3) for each
            time step.
        fr: The frame rate.
        save_path: The path to save the animation.
    """
    num_samples = next(iter(data.values())).shape[0]
    interval = 1 / fr

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    plots = {}
    for key, value in data.items():
        if value.shape[0] != num_samples:
            raise ValueError(
                f"Dot {key} has {value.shape[0]} samples, but expected {num_samples}"
            )
        plot = ax.scatter([], [], [])  # type: ignore
        plot.set_label(key)
        plots[key] = plot

    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_zlim(min_z, max_z)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    def init():
        for key, plot in plots.items():
            plot._offsets3d = (
                data[key][0, 2].reshape(1),
                data[key][0, 0].reshape(1),
                data[key][0, 1].reshape(1),
            )  # type: ignore
        return tuple(plots.values())

    def animate(i):
        idx = int(i * interval * freq)
        for key, plot in plots.items():
            plot._offsets3d = (
                data[key][idx, 2].reshape(1),
                data[key][idx, 0].reshape(1),
                data[key][idx, 1].reshape(1),
            )  # type: ignore
        return tuple(plots.values())

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=int(num_samples / (interval * freq)),
        interval=interval * 1000,
        blit=True,
    )

    if save_path is None:
        plt.show()
    else:
        if save_path.endswith(".gif"):
            writer = animation.PillowWriter(fps=fr)
        elif save_path.endswith(".mp4"):
            writer = animation.FFMpegWriter(fps=fr)
        else:
            raise ValueError(f"Invalid file extension: {save_path}")

        anim.save(save_path, writer=writer)


def visualize_calibration(
    session_dir: os.PathLike,
    config: Mapping[str, Any],
):
    if not os.path.exists(session_dir):
        raise FileNotFoundError(
            f"Session directory not found at {session_dir}"
        )

    # Raw data
    logger.info("Visualizing raw data")
    eyelink_path = os.path.join(session_dir, "eyelink_sample.csv")
    markers_path = os.path.join(session_dir, "markers.csv")
    eyelink_df = pd.read_csv(eyelink_path, index_col=False)
    markers_df = pd.read_csv(markers_path, index_col=False)

    eyelink_freq = config["preprocess"]["eyelink_freq"]
    markers_freq = config["preprocess"]["markers_freq"]

    eyelink_df, markers_df = format_columns(
        eyelink_df, markers_df, eyelink_freq, markers_freq, marker_idx=0
    )

    plot_eyelink_markers(
        eyelink_df,
        freq=eyelink_freq,
        markers_df=markers_df,
        markers_freq=markers_freq,
        title="Raw data",
        save_path=os.path.join(session_dir, "raw.png"),
    )

    # Preprocessed data
    logger.info("Visualizing preprocessed data")
    preprocessed_path = os.path.join(
        session_dir, config["preprocess"]["filename"]
    )
    preprocessed_df = pd.read_csv(preprocessed_path)
    plot_eyelink_markers(
        preprocessed_df,
        freq=eyelink_freq,
        title="Preprocessed data",
        save_path=os.path.join(session_dir, "preprocessed.png"),
    )

    # Eyelink data
    logger.info("Animating eyelink data")
    left_eye = cast(np.ndarray, eyelink_df[["left_x", "left_y"]].values)
    right_eye = cast(np.ndarray, eyelink_df[["right_x", "right_y"]].values)
    animate_2d_dots(
        {"Left eye": left_eye, "Right eye": right_eye},
        freq=eyelink_freq,
        **config["visualize"]["eyelink_range"],
        save_path=os.path.join(session_dir, "eyelink.mp4"),
    )

    # Predicted data
    logger.info("Animating predicted marker data")
    results_path = os.path.join(session_dir, config["predictions"]["filename"])
    results_df = pd.read_csv(results_path)
    targets = cast(
        np.ndarray, results_df[["target_x", "target_y", "target_z"]].values
    )
    preds = cast(np.ndarray, results_df[["pred_x", "pred_y", "pred_z"]].values)

    animate_3d_dots(
        {"Target": targets, "Prediction": preds},
        freq=eyelink_freq,
        **config["visualize"]["markers_range"],
        save_path=os.path.join(session_dir, "predictions.mp4"),
    )


def main(args=None):
    import argparse

    import yaml

    parser = argparse.ArgumentParser(
        description="Visualize the calibration data"
    )
    parser.add_argument(
        "-d",
        "--session-dir",
        type=str,
        default=os.path.join(os.environ["ROS_BAG_DIR"], "latest"),
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default=os.path.join(
            os.environ["TABLETOP_DIR"], "config", "gaze_calibration.yaml"
        ),
    )
    args = parser.parse_args(args)

    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s - %(message)s"
    )

    with open(args.config, "r") as f:
        config = cast(Mapping[str, Any], yaml.safe_load(f))

    visualize_calibration(args.session_dir, config)


if __name__ == "__main__":
    main()
