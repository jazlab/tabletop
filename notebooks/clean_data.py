import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch

pd.options.mode.copy_on_write = True

EYELINK_MISSING = -32768


def load_data(
    eyelink_csv_path: str, markers_csv_path: str
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Loads the eye tracker and optical marker data from the session directory.

    Args:
        eyelink_csv_path (str): Path to the eye tracker data file.
        markers_csv_path (str): Path to the optical marker data file.

    Returns:
        tuple[pd.DataFrame, pd.DataFrame]: A tuple containing the eye tracker and optical marker data.
    """
    eyelink_df = pd.read_csv(eyelink_csv_path)
    markers_df = pd.read_csv(markers_csv_path)

    return eyelink_df, markers_df


def verify_timestamps(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    eyelink_freq: float = 1000,
    markers_freq: float = 120,
    freq_rtol: float = 1e-3,
    max_marker_time_correction: float = 1e-3,
):
    """
    Verifies the integrity of the timestamps.

    Args:
        eyelink_df (pd.DataFrame): The eye tracker data.
        markers_df (pd.DataFrame): The optical marker data.
    """
    eyelink_times = eyelink_df[["bag_time", "time", "eyelink_time"]]
    markers_times = markers_df[["bag_time", "time", "original_time"]]

    # Check that the frame number and times are monotonic increasing
    assert markers_df["frame_number"].is_monotonic_increasing
    for df, name in [(eyelink_times, "eyelink"), (markers_times, "markers")]:
        freq = eyelink_freq if name == "eyelink" else markers_freq
        for col in df.columns:
            if not df[col].is_monotonic_increasing:
                raise ValueError(
                    f"{col} is not monotonic increasing for {name}"
                )
            if not np.isclose(df[col].diff().mean(), 1 / freq, rtol=freq_rtol):
                raise ValueError(
                    f"{col} diff is not close to the expected frequency {freq} for {name}"
                )
            if not np.isclose(df[col].diff().std(), 0, atol=1e-6):
                raise ValueError(
                    f"{col} diff std is not close to 0 for {name}"
                )

    # Check that the marker time is within the expected range
    d = markers_df.time - markers_df.original_time
    if d.min() < 0:
        raise ValueError(
            f"Marker time correction is negative: {d.min():.4f} < 0"
        )
    if d.max() > max_marker_time_correction:
        raise ValueError(
            f"Marker time correction is too large: {d.max():.4f} > {max_marker_time_correction:.4f}"
        )


def format_timestamps(
    eyelink_df: pd.DataFrame, markers_df: pd.DataFrame
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Formats the timestamps to seconds and checks for monotonicity.

    Args:
        eyelink_df (pd.DataFrame): The eye tracker data.
        markers_df (pd.DataFrame): The optical marker data.

    Returns:
        tuple[pd.DataFrame, pd.DataFrame]: A tuple containing the formatted eye tracker and optical marker data.
    """
    eyelink_df = eyelink_df.copy(deep=True)
    markers_df = markers_df.copy(deep=True)

    eyelink_df["bag_time"] = eyelink_df["bag_time_ns"] / 1e9
    markers_df["bag_time"] = markers_df["bag_time_ns"] / 1e9

    eyelink_df["time"] = (
        eyelink_df["header.stamp.sec"]
        + eyelink_df["header.stamp.nanosec"] / 1e9
    )
    markers_df["time"] = (
        markers_df["header.stamp.sec"]
        + markers_df["header.stamp.nanosec"] / 1e9
    )
    markers_df["original_time"] = (
        markers_df["header_original.stamp.sec"]
        + markers_df["header_original.stamp.nanosec"] / 1e9
    )

    eyelink_df["eyelink_time"] = eyelink_df["eyelink_time_ms"] / 1e3

    verify_timestamps(eyelink_df, markers_df)

    return eyelink_df, markers_df


def preprocess_data(
    eyelink_df: pd.DataFrame, markers_df: pd.DataFrame
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Preprocesses the data by selecting the relevant columns and dropping invalid rows.

    Args:
        eyelink_df (pd.DataFrame): The eye tracker data.
        markers_df (pd.DataFrame): The optical marker data.
    """

    eyelink_df = eyelink_df.copy(deep=True)
    markers_df = markers_df.copy(deep=True)

    eyelink_df = eyelink_df[
        [
            "time",
            "left_x",
            "left_y",
            "left_pupil",
            "right_x",
            "right_y",
            "right_pupil",
            "input",
        ]
    ]
    markers_df = markers_df[
        [
            "time",
            "marker_0_x",
            "marker_0_y",
            "marker_0_z",
        ]
    ]

    min_time = min(eyelink_df.time.min(), markers_df.time.min())
    eyelink_df["time"] = eyelink_df["time"] - min_time
    markers_df["time"] = markers_df["time"] - min_time

    eyelink_df["time_diff"] = eyelink_df["time"].diff()
    markers_df["time_diff"] = markers_df["time"].diff()

    for col in [
        "left_x",
        "left_y",
        "left_pupil",
        "right_x",
        "right_y",
        "right_pupil",
    ]:
        assert (eyelink_df[col] - eyelink_df[col].astype(int)).sum() == 0
        eyelink_df[col] = (
            eyelink_df[col].astype(int).replace(EYELINK_MISSING, np.nan)
        )

    eyelink_df = eyelink_df.dropna(
        subset=[
            "left_x",
            "left_y",
            "left_pupil",
            "right_x",
            "right_y",
            "right_pupil",
        ]
    )
    markers_df = markers_df.dropna(
        subset=["marker_0_x", "marker_0_y", "marker_0_z"]
    )
    markers_df["time_markers"] = markers_df.time

    return eyelink_df, markers_df


def merge_eyelink_markers(
    eyelink_df: pd.DataFrame, markers_df: pd.DataFrame
) -> pd.DataFrame:
    """
    Merges the eyelink and marker data on closest time.
    """
    merged_df = pd.merge_asof(
        eyelink_df,
        markers_df,
        on="time",
        direction="backward",
        tolerance=eyelink_df.diff.max(),
        suffixes=("_eyelink", "_markers"),
    )
    assert merged_df.shape[0] == eyelink_df.shape[0]
    matched = merged_df.time_markers.notna().sum()
    unique = (merged_df.time_markers.dropna() * 1000).astype(int).unique()

    print(
        f"Matched {matched.sum()} ({unique.shape[0]} unique) out of {markers_df.shape[0]} marker data points"
    )

    return merged_df


def remove_duplicates(merged_df: pd.DataFrame) -> pd.DataFrame:
    """
    Removes duplicated marker data as a result of the merge.
    """
    marker_cols = [col for col in merged_df.columns if "marker" in col]
    merged_markers = merged_df[marker_cols]
    diff = merged_markers.time_markers.diff()
    merged_markers[diff.notna() & (diff < 1e-6)] = np.nan
    merged_df[marker_cols] = merged_markers

    new_matched = merged_df.time_markers.notna()
    new_unique = (merged_df.time_markers.dropna() * 1000).astype(int).unique()
    assert (
        unique.shape[0] == new_unique.shape[0]
    ), f"Expected {unique.shape[0]} unique marker data points, got {new_unique.shape[0]}"
    assert (
        new_matched.sum() == new_unique.shape[0]
    ), f"Expected all {new_matched.sum()} matched marker data points to be unique, got {new_unique.shape[0]} unique"

    print(
        f"After removing duplicates, {new_matched.sum()} out of previous {matched.sum()} marker data points remain"
    )

    print(
        f"Marker time gaps before merging | "
        f"min: {merged_df.time_diff_markers.min():.4f}, "
        f"max: {merged_df.time_diff_markers.max():.4f}"
    )
    gaps = merged_df.time[merged_df.time_markers.notna()].diff()
    print(
        f"Marker time gaps after merging | "
        f"min: {gaps.min():.4f}, "
        f"max: {gaps.max():.4f}"
    )

    # Remove rows where the gap between marker datapoints is too large to be interpolated accurately
    asof = merged_df.asof(
        merged_df.index, subset=["marker_0_x", "marker_0_y", "marker_0_z"]
    )
    to_drop = asof.time[
        asof.time.isna()
        | (merged_df.time > asof.time + merged_df.time_diff_markers.bfill())
    ]
    drop_times = to_drop.unique()
    print(
        f"Found {to_drop.shape[0]} rows (forming {len(drop_times)} contiguous gaps) where "
        f"the time since the last marker datapoint is greater than the maximum expected gap: "
        f"{merged_df.time_diff_markers.max():.4f}"
    )

    merged_df = merged_df[~asof.time.isin(drop_times)]

    num_dropped = merged_df.shape[0] - merged_df.shape[0]
    print(f"Dropped {num_dropped} out of {merged_df.shape[0]} rows")

    # %% [markdown]
    # # Interpolate marker data onto eyelink time points

    # %%
    interp = no_gaps.copy(deep=True)
    interp.set_index("time", inplace=True)
    markers_cols = ["marker_0_x", "marker_0_y", "marker_0_z"]
    interp[markers_cols] = interp[markers_cols].interpolate(method="slinear")
    interp.reset_index(inplace=True)
    interp


# %% [markdown]
# # Clean up data for training

# %%
df = interp.copy(deep=True)
df = df.drop(
    columns=[
        "left_pupil",
        "right_pupil",
        "input",
        "time_diff_eyelink",
        "time_diff_markers",
        "time_markers",
    ]
)
df.rename(
    columns={
        "marker_0_x": "marker_x",
        "marker_0_y": "marker_y",
        "marker_0_z": "marker_z",
    }
)
df.to_csv(os.path.join(session_dir, "calibration_data.csv"), index=False)

# %%
X = torch.tensor(df[["left_x", "left_y", "right_x", "right_y"]].values)
Y = torch.tensor(df[["marker_0_x", "marker_0_y", "marker_0_z"]].values)

torch.save(X, os.path.join(session_dir, "eyelink.pt"))
torch.save(Y, os.path.join(session_dir, "markers.pt"))


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

    for col in ["marker_0_x", "marker_0_y", "marker_0_z"]:
        ax[i].plot(markers_df.time, markers_df[col])
        ax[i].set_title(f"Markers {col}")
        i += 1

    plot_eyelink_markers(eyelink_df, markers_df)
    plt.savefig(os.path.join(session_dir, "eyelink_markers.png"))
    plt.show()
    plot_eyelink_markers(interp, interp)
    plt.savefig(os.path.join(session_dir, "eyelink_markers_interp.png"))
    plt.show()


# %%
def animate_eyelink(eyelink_df, fr=10, save_path="eyelink.gif"):
    freq = 1 / eyelink_df.time.diff().median()
    interval = 1 / fr

    fig, ax = plt.subplots()
    (points,) = ax.plot([], [], "o")
    ax.set_xlim(-10000, 10000)
    ax.set_ylim(-15000, 15000)

    def init():
        row = df.iloc[0]
        left_x, left_y, right_x, right_y = row[
            ["left_x", "left_y", "right_x", "right_y"]
        ]
        points.set_data([left_x, right_x], [left_y, right_y])
        return (points,)

    def animate(i):
        row = df.iloc[int(i * interval * freq)]
        left_x, left_y, right_x, right_y = row[
            ["left_x", "left_y", "right_x", "right_y"]
        ]
        points.set_data([left_x, right_x], [left_y, right_y])
        return (points,)

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=int(df.shape[0] / (interval * freq)),
        interval=interval * 1000,
        blit=True,
    )

    if save_path.endswith(".gif"):
        writer = animation.PillowWriter(fps=fr)
    elif save_path.endswith(".mp4"):
        writer = animation.FFMpegWriter(fps=fr)
    else:
        raise ValueError(f"Invalid file extension: {save_path}")

    anim.save(save_path, writer=writer)


animate_eyelink(
    eyelink_df, fr=10, save_path=os.path.join(session_dir, "eyelink.mp4")
)
animate_eyelink(
    interp, fr=10, save_path=os.path.join(session_dir, "eyelink_interp.mp4")
)


# %%
def animate_markers(df, fr=10, save_path="markers.gif"):
    freq = 1 / df.time.diff().median()
    interval = 1 / fr

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    (points,) = ax.plot([], [], [], "o")
    ax.set_xlim(df.marker_0_x.min(), df.marker_0_x.max())
    ax.set_ylim(df.marker_0_y.min(), df.marker_0_y.max())
    ax.set_zlim(df.marker_0_z.min(), df.marker_0_z.max())

    def init():
        x, y, z = df.iloc[0][["marker_0_x", "marker_0_y", "marker_0_z"]]
        points.set_data([x], [y])
        points.set_3d_properties([z], "z")
        return (points,)

    def animate(i):
        row = df.iloc[int(i * interval * freq)]
        x, y, z = row[["marker_0_x", "marker_0_y", "marker_0_z"]]
        points.set_data([x], [y])
        points.set_3d_properties([z], "z")
        return (points,)

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=int(df.shape[0] / (interval * freq)),
        interval=interval * 1000,
        blit=True,
    )

    if save_path.endswith(".gif"):
        writer = animation.PillowWriter(fps=fr)
    elif save_path.endswith(".mp4"):
        writer = animation.FFMpegWriter(fps=fr)
    else:
        raise ValueError(f"Invalid file extension: {save_path}")

    anim.save(save_path, writer=writer)


animate_markers(
    markers_df, fr=10, save_path=os.path.join(session_dir, "markers.mp4")
)
animate_markers(
    interp, fr=10, save_path=os.path.join(session_dir, "markers_interp.mp4")
)
