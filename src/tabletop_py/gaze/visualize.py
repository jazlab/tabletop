"""Visualization tools for gaze estimation data and results.

This module provides plotting and animation functions for visualizing
eye tracking data, marker positions, and model predictions.

Functions:
    plot_eyelink_markers: Time series plots of eye tracking and marker data.
    animate_2d_dots: 2D scatter animation of eye positions.
    animate_3d_dots: 3D scatter animation of gaze points.
    visualize_calibration: Full visualization pipeline for a session.
    main: CLI entry point.

Supported Output Formats:
    - Static plots: PNG, PDF, SVG (via matplotlib)
    - Animations: MP4 (FFMpeg), GIF (Pillow)

Example:
    # Visualize raw and preprocessed data
    plot_eyelink_markers(df, "Raw Data", freq=500, save_path="raw.png")

    # Animate predictions vs targets
    animate_3d_dots(
        {"Target": targets, "Prediction": preds},
        freq=500, min_x=-0.5, max_x=0.5, ...
    )
"""

import logging
import os
from collections.abc import Mapping
from typing import Any, Literal, Optional, cast

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

logger = logging.getLogger(__name__)


def reindex_asof(
    df: pd.DataFrame,
    new_idx: np.ndarray | pd.Series,
    on: str,
    *,
    direction: Literal["backward", "forward", "nearest"] = "backward",
    tolerance: Optional[float] = None,
) -> pd.DataFrame:
    return pd.merge_asof(
        pd.DataFrame({on: new_idx}),
        df,
        on=on,
        direction=direction,
        tolerance=tolerance,  # pyright: ignore[reportArgumentType]
    )


def plot_eyelink_markers(
    df: pd.DataFrame,
    title: str,
    *,
    markers_df: Optional[pd.DataFrame] = None,
    reindex: bool = True,
    freq: Optional[float] = None,
    markers_freq: Optional[float] = None,
    plot_type: Literal["line", "scatter"] = "line",
    scatter_point_size: float = 0.5,
    overlay: bool = False,
    save_path: Optional[str] = None,
):
    """
    Plot the eyelink and markers data.

    Args:
        df: The dataframe containing the eyelink data (and optionally markers data).
        markers_df: Optional dataframe containing the markers data if not provided in df.
        title: The title of the plot.
        save_path: The path to save the plot.
    """
    if plot_type not in ("line", "scatter"):
        raise ValueError(f"plot_type must be line or scatter, got {plot_type}")

    if reindex:
        if freq is None:
            raise ValueError("freq must be provided if reindex is True")

        steady_idx = np.arange(df["time"].min(), df["time"].max(), 1 / freq)
        df = reindex_asof(
            df, steady_idx, on="time", direction="backward", tolerance=1 / freq
        )

        if markers_df is not None:
            if markers_freq is None:
                raise ValueError(
                    "markers_freq must be provided if markers_df is provided"
                )

            steady_idx = np.arange(
                markers_df["time"].min(),
                markers_df["time"].max(),
                1 / markers_freq,
            )
            markers_df = reindex_asof(
                markers_df,
                steady_idx,
                on="time",
                direction="backward",
                tolerance=1 / markers_freq,
            )

    eyelink_df = df

    if markers_df is None:
        markers_df = eyelink_df

    if overlay:
        fig, ax = plt.subplots(1, 1, figsize=(10, 3))
        ax = [ax]
    else:
        fig, ax = plt.subplots(5, 1, sharex=True, figsize=(10, 10))

    fig.suptitle(title)

    i = 0
    for dim in ["x", "y"]:
        for col in [f"left_{dim}", f"right_{dim}"]:
            # Normalize the data if overlaying so scales are comparable
            if overlay:
                data = (eyelink_df[col] - eyelink_df[col].mean()) / eyelink_df[
                    col
                ].std()
            else:
                data = eyelink_df[col]

            if plot_type == "line":
                ax[i].plot(
                    eyelink_df["time"],
                    data,
                    label=col,
                    alpha=0.5,
                    linestyle="--" if overlay else "-",
                )
            else:
                ax[i].scatter(
                    eyelink_df["time"],
                    data,
                    label=col,
                    s=scatter_point_size,
                    alpha=0.5,
                )
        if not overlay:
            ax[i].set_title(f"Eyelink {dim.upper()}")
            ax[i].legend()
            ax[i].grid(True, which="both", axis="x")
            i += 1

    for col in ["marker_x", "marker_y", "marker_z"]:
        # Normalize the data if overlaying so scales are comparable
        if overlay:
            data = (markers_df[col] - markers_df[col].mean()) / markers_df[
                col
            ].std()
        else:
            data = markers_df[col]

        if plot_type == "line":
            ax[i].plot(
                markers_df["time"],
                data,
                label=col if overlay else None,
                alpha=0.5 if overlay else 1,
                linestyle="--" if overlay else "-",
            )
        else:
            ax[i].scatter(
                markers_df["time"],
                data,
                label=col if overlay else None,
                s=scatter_point_size,
                alpha=0.5 if overlay else 1,
            )

        if not overlay:
            ax[i].set_title(f"Markers {col}")
            ax[i].grid(True, which="both", axis="x")
            i += 1

    if overlay:
        ax[0].legend()

    fig.tight_layout()

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
    save_path: Optional[str] = None,
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
    title: str | None = None,
    freq: float,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    min_z: float,
    max_z: float,
    fr: int = 10,
    rotate_to_world: bool = False,
    save_path: Optional[str] = None,
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

    if title is not None:
        ax.set_title(title)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_zlim(min_z, max_z)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    if rotate_to_world:
        x_idx = 2
        y_idx = 0
        z_idx = 1
    else:
        x_idx = 0
        y_idx = 1
        z_idx = 2

    def init():
        for key, plot in plots.items():
            plot._offsets3d = (
                data[key][0, x_idx].reshape(1),
                data[key][0, y_idx].reshape(1),
                data[key][0, z_idx].reshape(1),
            )  # type: ignore
        return tuple(plots.values())

    def animate(i):
        idx = int(i * interval * freq)
        for key, plot in plots.items():
            plot._offsets3d = (
                data[key][idx, x_idx].reshape(1),
                data[key][idx, y_idx].reshape(1),
                data[key][idx, z_idx].reshape(1),
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
    session_dir: os.PathLike, config: Mapping[str, Any] | os.PathLike | str
):
    if not os.path.exists(session_dir):
        raise FileNotFoundError(
            f"Session directory not found at {session_dir}"
        )

    if not isinstance(config, Mapping):
        with open(config, "r") as f:
            config = cast(Mapping[str, Any], yaml.safe_load(f))

    # Get frequencies
    eyelink_freq = config["eyelink_freq"]
    markers_freq = config["markers_freq"]
    preprocess_filename = config["preprocess"]["filename"]
    predictions_filename = config["predictions"]["filename"]
    config = cast(Mapping[str, Any], config["visualize"])

    # Raw data
    logger.info("Visualizing raw data")
    eyelink_path = os.path.join(session_dir, "raw_eyelink.csv")
    markers_path = os.path.join(session_dir, "raw_markers.csv")
    eyelink_df = pd.read_csv(eyelink_path, index_col=False)
    markers_df = pd.read_csv(markers_path, index_col=False)
    plot_eyelink_markers(
        eyelink_df,
        title="Raw data",
        freq=eyelink_freq,
        markers_df=markers_df,
        markers_freq=markers_freq,
        save_path=os.path.join(session_dir, "raw.png"),
    )

    # Preprocessed data
    logger.info("Visualizing preprocessed data")
    preprocessed_path = os.path.join(session_dir, preprocess_filename)
    preprocessed_df = pd.read_csv(preprocessed_path)
    plot_eyelink_markers(
        preprocessed_df,
        title="Preprocessed data",
        freq=eyelink_freq,
        save_path=os.path.join(session_dir, "preprocessed.png"),
    )

    # Eyelink data
    logger.info("Animating eyelink data")
    left_eye = cast(
        np.ndarray, preprocessed_df[["left_x", "left_y"]].to_numpy()
    )
    right_eye = cast(
        np.ndarray, preprocessed_df[["right_x", "right_y"]].to_numpy()
    )
    animate_2d_dots(
        {"Left eye": left_eye, "Right eye": right_eye},
        freq=eyelink_freq,
        **config["animate_2d_dots"],
        save_path=os.path.join(session_dir, "eyelink.mp4"),
    )

    # Predicted data
    logger.info("Animating predicted marker data")
    results_path = os.path.join(session_dir, predictions_filename)
    results_df = pd.read_csv(results_path)
    targets = cast(
        np.ndarray, results_df[["target_x", "target_y", "target_z"]].to_numpy()
    )
    preds = cast(
        np.ndarray, results_df[["pred_x", "pred_y", "pred_z"]].to_numpy()
    )

    animate_3d_dots(
        {"Target": targets, "Prediction": preds},
        freq=eyelink_freq,
        **config["animate_3d_dots"],
        save_path=os.path.join(session_dir, "predictions.mp4"),
    )


def main(args=None):
    """Entry point for visualizing gaze calibration results.

    CLI for generating plots and animations of raw data, preprocessed
    data, and model predictions from a calibration session.
    """
    import argparse

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
            os.environ["TABLETOP_DIR"], "config", "gaze_estimation.yaml"
        ),
    )
    args = parser.parse_args(args)

    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s - %(message)s"
    )

    visualize_calibration(**vars(args))


if __name__ == "__main__":
    main()
