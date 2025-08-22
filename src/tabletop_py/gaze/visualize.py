import os
from collections.abc import Mapping
from typing import Any, cast

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_eyelink_markers(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    title: str,
    save_path: str | None = None,
):
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
    freq: float,
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
        fr: The frame rate.
        save_path: The path to save the animation.
    """
    num_samples = next(iter(data.values())).shape[0]
    interval = 1 / fr

    fig = plt.figure()
    ax = fig.add_subplot()

    plots = {}
    colors = np.linspace(0, 1, len(data))
    min_x = np.inf
    min_y = np.inf
    max_x = -np.inf
    max_y = -np.inf
    for i, (key, value) in enumerate(data.items()):
        if value.shape != (num_samples, 2):
            raise ValueError(
                f"Dot {key} has {value.shape} samples, but expected {(num_samples, 2)}"
            )
        plot = ax.scatter([], [])
        plot.set_label(key)
        plots[key] = plot
        min_x = min(min_x, value[:, 0].min())
        min_y = min(min_y, value[:, 1].min())
        max_x = max(max_x, value[:, 0].max())
        max_y = max(max_y, value[:, 1].max())

    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)

    def init():
        for key, plot in plots.items():
            plot.set_offsets(data[key][0])
            # plot.set_cmap("tab10")
            # plot.set_array(colors[i])
        return tuple(plots.values())

    def animate(i):
        idx = int(i * interval * freq)
        for key, plot in plots.items():
            plot.set_offsets(data[key][idx])
            # plot.set_cmap("tab10")
            # plot.set_array(colors[i])
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
    freq: float,
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
    colors = np.linspace(0, 1, len(data))
    min_x = np.inf
    min_y = np.inf
    min_z = np.inf
    max_x = -np.inf
    max_y = -np.inf
    max_z = -np.inf
    for i, (key, value) in enumerate(data.items()):
        if value.shape[0] != num_samples:
            raise ValueError(
                f"Dot {key} has {value.shape[0]} samples, but expected {num_samples}"
            )
        plot = ax.scatter([], [], [])  # type: ignore
        plot.set_label(key)
        plots[key] = plot

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(0.0, 1.0)
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
            # plot.set_cmap("tab10")
            # plot.set_array(colors)
        return tuple(plots.values())

    def animate(i):
        idx = int(i * interval * freq)
        for key, plot in plots.items():
            plot._offsets3d = (
                data[key][idx, 2].reshape(1),
                data[key][idx, 0].reshape(1),
                data[key][idx, 1].reshape(1),
            )  # type: ignore
            # plot.set_cmap("tab10")
            # plot.set_array(colors)
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

    # Preprocessed data
    preprocessed_path = os.path.join(
        session_dir, config["preprocess"]["filename"]
    )
    preprocessed_df = pd.read_csv(preprocessed_path)
    eyelink_df = cast(
        pd.DataFrame,
        preprocessed_df[["time", "left_x", "left_y", "right_x", "right_y"]],
    )
    markers_df = cast(
        pd.DataFrame,
        preprocessed_df[["time", "marker_x", "marker_y", "marker_z"]],
    )
    plot_eyelink_markers(
        eyelink_df,
        markers_df,
        title="Preprocessed data",
        save_path=os.path.join(session_dir, "calibration.png"),
    )

    left_eye = cast(np.ndarray, eyelink_df[["left_x", "left_y"]].values)
    right_eye = cast(np.ndarray, eyelink_df[["right_x", "right_y"]].values)
    # animate_2d_dots(
    #     {"Left eye": left_eye, "Right eye": right_eye},
    #     1000,
    #     save_path=os.path.join(session_dir, "eyelink.mp4"),
    # )

    # Predicted data
    results_path = os.path.join(session_dir, config["predictions"]["filename"])
    results_df = pd.read_csv(results_path)
    targets = cast(
        np.ndarray, results_df[["target_x", "target_y", "target_z"]].values
    )
    preds = cast(np.ndarray, results_df[["pred_x", "pred_y", "pred_z"]].values)

    animate_3d_dots(
        {"Target": targets, "Prediction": preds},
        config["preprocess"]["eyelink_freq"],
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

    with open(args.config, "r") as f:
        config = cast(Mapping[str, Any], yaml.safe_load(f))

    visualize_calibration(args.session_dir, config)


if __name__ == "__main__":
    main()
