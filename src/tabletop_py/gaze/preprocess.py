import os
from typing import cast

import numpy as np
import pandas as pd

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
    max_marker_time_correction: float = 0.005,
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
    for df, df_name in [
        (eyelink_times, "eyelink"),
        (markers_times, "markers"),
    ]:
        freq = eyelink_freq if df_name == "eyelink" else markers_freq
        for col_name in df.columns:
            col = df[col_name]
            assert isinstance(col, pd.Series)
            if not col.is_monotonic_increasing:
                raise ValueError(
                    f"{col_name} is not monotonic increasing for {df_name}"
                )
            diff_mean = col.diff().mean()
            if not np.isclose(diff_mean, 1 / freq, rtol=freq_rtol):
                raise ValueError(
                    f"{col_name} diff mean of {diff_mean:.4f} is not close to the expected frequency {freq} for {df_name}"
                )
            diff_std = col.diff().std()
            if not np.isclose(diff_std, 0, atol=diff_mean * 0.5):
                raise ValueError(
                    f"{col_name} diff std of {diff_std:.4f} is not close to 0 for {df_name}"
                )

    # Check that the marker time is within the expected range
    correction = markers_df["original_time"] - markers_df["time"]
    if correction.min() < 0:
        raise ValueError(
            f"Marker time correction is negative for {correction[correction < 0].shape[0]} out of {correction.shape[0]} rows, with min: {correction.min():.4f}"
        )
    if correction.max() > max_marker_time_correction:
        raise ValueError(
            f"Marker time correction is too large: {correction.max():.4f} > {max_marker_time_correction:.4f}"
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

    eyelink_df["bag_time"] = eyelink_df.bag_time_ns / 1e9
    markers_df["bag_time"] = markers_df.bag_time_ns / 1e9

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


def clean_data(
    eyelink_df: pd.DataFrame, markers_df: pd.DataFrame, marker_idx: int = 0
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Cleans the data by selecting the relevant columns and dropping invalid rows.

    Args:
        eyelink_df (pd.DataFrame): The eye tracker data.
        markers_df (pd.DataFrame): The optical marker data.
    """

    eyelink_df = eyelink_df.copy(deep=True)
    markers_df = markers_df.copy(deep=True)

    if f"markers[{marker_idx}].id_type" not in markers_df.columns:
        raise ValueError(
            f"Marker {marker_idx} not found in markers_df. Columns: {markers_df.columns}"
        )
    markers_df[["marker_x", "marker_y", "marker_z"]] = markers_df[
        [
            f"markers[{marker_idx}].translation.x",
            f"markers[{marker_idx}].translation.y",
            f"markers[{marker_idx}].translation.z",
        ]
    ]

    eyelink_df = cast(
        pd.DataFrame,
        eyelink_df[
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
        ],
    )
    markers_df = cast(
        pd.DataFrame,
        markers_df[
            [
                "time",
                "marker_x",
                "marker_y",
                "marker_z",
            ]
        ],
    )

    min_time = min(eyelink_df["time"].min(), markers_df["time"].min())
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
    markers_df = markers_df.dropna(subset=["marker_x", "marker_y", "marker_z"])
    markers_df["time_markers"] = markers_df["time"]

    return eyelink_df, markers_df


def merge_eyelink_markers(
    eyelink_df: pd.DataFrame, markers_df: pd.DataFrame
) -> pd.DataFrame:
    """
    Merges the eyelink and marker data on closest time.
    """
    df = pd.merge_asof(
        eyelink_df,
        markers_df,
        on="time",
        direction="backward",
        tolerance=eyelink_df["time_diff"].max(),  # type: ignore
        suffixes=("_eyelink", "_markers"),
    )
    assert df.shape[0] == eyelink_df.shape[0]
    matched = df["time_markers"].notna().sum()
    unique = (df["time_markers"].dropna() * 1000).astype(int).unique()

    print(
        f"Matched {matched.sum()} ({unique.shape[0]} unique) out of {markers_df.shape[0]} marker data points"
    )

    marker_cols = [col for col in df.columns if "marker" in col]
    merged_markers = df[marker_cols]
    diff = merged_markers["time_markers"].diff()  # type: ignore
    merged_markers[diff.notna() & (diff < 1e-6)] = np.nan

    df[marker_cols] = merged_markers

    new_matched = df["time_markers"].notna()
    new_unique = (df["time_markers"].dropna() * 1000).astype(int).unique()
    assert (
        unique.shape[0] == new_unique.shape[0]
    ), f"Expected {unique.shape[0]} unique marker data points after removing duplicates, got {new_unique.shape[0]}"
    assert (
        new_matched.sum() == new_unique.shape[0]
    ), f"Expected all {new_matched.sum()} matched marker data points to be unique, got {new_unique.shape[0]} unique"

    print(
        f"After removing duplicates, {new_matched.sum()} out of previous {matched.sum()} marker data points remain"
    )

    return df


def interpolate_markers(df: pd.DataFrame) -> pd.DataFrame:
    """
    Interpolates the marker data onto the eyelink time points.
    """
    df = df.copy(deep=True)

    print(
        f"Marker time gaps before merging | "
        f"min: {df['time_diff_markers'].min():.4f}, "
        f"max: {df['time_diff_markers'].max():.4f}"
    )
    gaps = df["time"][df["time_markers"].notna()].diff()  # type: ignore
    print(
        f"Marker time gaps after merging | "
        f"min: {gaps.min():.4f}, "
        f"max: {gaps.max():.4f}"
    )

    # Remove rows where the gap between marker datapoints is too large to be interpolated accurately
    asof = cast(
        pd.DataFrame,
        df.asof(df.index, subset=["marker_x", "marker_y", "marker_z"]),
    )
    to_drop = asof["time"][
        asof["time"].isna()
        | (df["time"] > asof["time"] + df["time_diff_markers"].bfill())
    ]
    drop_times = to_drop.unique()  # type: ignore
    print(
        f"Found {to_drop.shape[0]} rows (forming {len(drop_times)} contiguous gaps) where "
        f"the time since the last marker datapoint is greater than the maximum expected gap: "
        f"{df['time_diff_markers'].max():.4f}"
    )

    df = cast(pd.DataFrame, df[~asof["time"].isin(drop_times)])  # type: ignore

    num_dropped = asof.shape[0] - df.shape[0]
    print(f"Dropped {num_dropped} out of {df.shape[0]} rows")

    df.set_index("time", inplace=True)
    markers_cols = ["marker_x", "marker_y", "marker_z"]
    df[markers_cols] = df[markers_cols].interpolate(method="slinear")
    df = df.dropna(subset=markers_cols)
    df.reset_index(inplace=True)

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

    return df


def preprocess_data(session_dir: str, save: bool = False) -> pd.DataFrame:
    """
    Preprocesses the data by cleaning and merging the eye tracker and optical marker data.
    """
    eyelink_df, markers_df = load_data(
        os.path.join(session_dir, "eyelink_sample.csv"),
        os.path.join(session_dir, "markers.csv"),
    )
    eyelink_df, markers_df = format_timestamps(eyelink_df, markers_df)
    eyelink_df, markers_df = clean_data(eyelink_df, markers_df, marker_idx=0)
    df = merge_eyelink_markers(eyelink_df, markers_df)
    df = interpolate_markers(df)

    if save:
        df.to_csv(
            os.path.join(session_dir, "calibration_data.csv"), index=False
        )

    return df


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="Clean the eye tracker and optical marker data."
    )
    parser.add_argument(
        "-d",
        "--session-dir",
        type=str,
        default=os.path.join(os.environ["ROS_BAG_DIR"], "latest"),
        help="The path to the session directory.",
    )
    args = parser.parse_args(args)

    preprocess_data(args.session_dir, save=True)


if __name__ == "__main__":
    main()
