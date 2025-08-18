import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_eyelink_markers(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    save_path: str | None = None,
):
    fig, ax = plt.subplots(7, 1, sharex=True, figsize=(10, 15))
    i = 0
    for col in ["left_x", "left_y", "right_x", "right_y"]:
        ax[i].plot(eyelink_df.time, eyelink_df[col])
        ax[i].set_title(f"Eyelink {col}")
        i += 1

    for col in ["marker_x", "marker_y", "marker_z"]:
        ax[i].plot(markers_df.time, markers_df[col])
        ax[i].set_title(f"Markers {col}")
        i += 1

    if save_path is not None:
        plt.savefig(save_path)
    else:
        plt.show()


def animate_2d_dots(
    data: np.ndarray,
    freq: float,
    fr: int = 10,
    save_path: str | None = None,
):
    """
    Animates 2D dots.

    Args:
        data: The data to animate. Should be an array of size (T, N, 2) where T is the number of time steps and N is the number of dots.
        freq: The frequency of the data.
        fr: The frame rate.
        save_path: The path to save the animation.
    """
    interval = 1 / fr

    fig = plt.figure()
    ax = fig.add_subplot()

    points = ax.scatter([], [])
    ax.set_xlim(data[:, :, 0].min(), data[:, :, 0].max())
    ax.set_ylim(data[:, :, 1].min(), data[:, :, 1].max())
    colors = np.linspace(0, 1, data.shape[1])

    def init():
        points.set_offsets(data[0, :, :])
        points.set_cmap("tab10")
        points.set_array(colors)
        return (points,)

    def animate(i):
        idx = int(i * interval * freq)
        points.set_offsets(data[idx, :, :])
        points.set_cmap("tab10")
        points.set_array(colors)
        return (points,)

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=int(data.shape[0] / (interval * freq)),
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
    data: np.ndarray,
    freq: float,
    fr: int = 10,
    save_path: str | None = None,
):
    """
    Animates 3D dots.

    Args:
        data: The data to animate. Should be an array of size (T, N, 3) where T is the number of time steps and N is the number of dots.
        fr: The frame rate.
        save_path: The path to save the animation.
    """
    interval = 1 / fr

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    points = ax.scatter([], [], [])  # type: ignore
    ax.set_xlim(data[:, :, 0].min(), data[:, :, 0].max())
    ax.set_ylim(data[:, :, 1].min(), data[:, :, 1].max())
    ax.set_zlim(data[:, :, 2].min(), data[:, :, 2].max())
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    colors = np.linspace(0, 1, data.shape[1])

    def init():
        points._offsets3d = (data[0, :, 0], data[0, :, 1], data[0, :, 2])  # type: ignore
        points.set_cmap("tab10")
        points.set_array(colors)
        return (points,)

    def animate(i):
        idx = int(i * interval * freq)
        points._offsets3d = (  # type: ignore
            data[idx, :, 0],
            data[idx, :, 1],
            data[idx, :, 2],
        )
        points.set_cmap("tab10")
        points.set_array(colors)
        return (points,)

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=int(data.shape[0] / (interval * freq)),
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
