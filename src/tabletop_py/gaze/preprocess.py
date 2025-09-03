import logging
import os
from collections.abc import Mapping
from typing import Any, Optional, cast

import numpy as np
import pandas as pd
from scipy import stats
from scipy.signal import savgol_filter

logger = logging.getLogger(__name__)

pd.options.mode.copy_on_write = True

EYELINK_MISSING = -32768


def reindex_and_interpolate(
    df: pd.DataFrame,
    new_idx: np.ndarray | pd.Series,
    on: str,
    direction: str = "backward",
    tolerance: Optional[float] = None,
) -> pd.DataFrame:
    """
    Merges two dataframes on a common column with a tolerance.
    """
    new_df = pd.DataFrame({on: new_idx})
    new_df = pd.merge_asof(
        new_df,
        df,
        on=on,
        direction=direction,
        tolerance=tolerance,  # type: ignore
    )
    data_cols = [col for col in df.columns if col != on]
    isna = new_df[data_cols].isna().any(axis=1)

    for col in df.columns:
        if col == on:
            continue
        data = np.interp(new_df[on], df[on], df[col])
        data[isna] = np.nan  # type: ignore
        new_df[col] = data

    return new_df


def reindex_steady_time(
    df: pd.DataFrame,
    freq: float,
    on: str,
    direction: str = "backward",
    tolerance: Optional[float] = None,
) -> pd.DataFrame:
    """
    Interpolates the data with NaNs.
    """
    steady_idx = np.arange(df[on].min(), df[on].max(), 1 / freq)
    if tolerance is None:
        tolerance = (1 + 1e-1) / freq
    return reindex_and_interpolate(
        df, steady_idx, on=on, direction=direction, tolerance=tolerance
    )


def verify_timestamps(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    eyelink_freq: float,
    markers_freq: float,
    freq_rtol: float = 1e-3,
    max_marker_time_correction: float = 0.01,
):
    """
    Verifies the integrity of the timestamps.

    Args:
        eyelink_df (pd.DataFrame): The eye tracker data.
        markers_df (pd.DataFrame): The optical marker data.
        eyelink_freq (float): The frequency of the eye tracker data.
        markers_freq (float): The frequency of the optical marker data.
        freq_rtol (float): The relative tolerance for the frequency.
        max_marker_time_correction (float): The maximum allowed time correction for the optical marker data.
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


def format_columns(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    eyelink_freq: float,
    markers_freq: float,
    marker_idx: int,
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

    verify_timestamps(eyelink_df, markers_df, eyelink_freq, markers_freq)

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

    return eyelink_df, markers_df


def clean_data(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    start_time: float = 0.0,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Cleans the data by selecting the relevant columns and dropping invalid rows.

    Args:
        eyelink_df (pd.DataFrame): The eye tracker data.
        markers_df (pd.DataFrame): The optical marker data.
    """

    eyelink_df = eyelink_df.copy(deep=True)
    markers_df = markers_df.copy(deep=True)

    min_time = min(eyelink_df["time"].min(), markers_df["time"].min())
    eyelink_df["time"] = eyelink_df["time"] - min_time
    markers_df["time"] = markers_df["time"] - min_time

    eyelink_df = cast(
        pd.DataFrame,
        eyelink_df[eyelink_df["time"] >= start_time],
    )
    markers_df = cast(
        pd.DataFrame,
        markers_df[markers_df["time"] >= start_time],
    )

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

    eyelink_zscores = stats.zscore(
        eyelink_df[["left_x", "left_y", "right_x", "right_y"]]
    )
    eyelink_df = eyelink_df[(np.abs(eyelink_zscores) < 3.0).all(axis=1)]  # type: ignore
    markers_zscores = stats.zscore(
        markers_df[["marker_x", "marker_y", "marker_z"]]
    )
    markers_df = markers_df[(np.abs(markers_zscores) < 3.0).all(axis=1)]  # type: ignore

    return eyelink_df, markers_df


def merge_eyelink_markers(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    eyelink_freq: float,
) -> pd.DataFrame:
    """
    Merges the eyelink and marker data on closest time.
    """
    df = reindex_steady_time(eyelink_df, eyelink_freq, on="time")
    df = df.dropna()
    markers_df = reindex_and_interpolate(
        markers_df,
        df["time"],  # type: ignore
        on="time",
        tolerance=1 / eyelink_freq,
    )
    num_rows = df.shape[0]

    df = pd.merge(df, markers_df, on="time", suffixes=("_eyelink", "_marker"))

    assert (
        df.shape[0] == num_rows
    ), f"Expected {num_rows} rows after merging, got {df.shape[0]}"
    marker_cols = [col for col in df.columns if "marker" in col]
    matched = df[marker_cols].notna().all(axis=1).sum()  # type: ignore

    logger.info(
        f"Matched {matched} out of {markers_df.shape[0]} marker data points"
    )

    return df


def interpolate_markers(
    df: pd.DataFrame, max_marker_gap: float
) -> pd.DataFrame:
    """
    Interpolates the marker data onto the eyelink time points.
    """
    df = df.copy(deep=True)

    marker_cols = [col for col in df.columns if "marker" in col]
    gaps = df["time"][df[marker_cols].notna().all(axis=1)].diff()  # type: ignore
    logger.info(
        f"Marker time gaps after merging | "
        f"min: {gaps.min():.4f}, "
        f"max: {gaps.max():.4f}"
    )

    # Remove rows where the gap between marker datapoints is too large to be interpolated accurately
    asof = cast(
        pd.DataFrame,
        df.asof(df.index, subset=marker_cols),
    )
    to_drop = asof["time"][
        asof["time"].isna() | (df["time"] > (asof["time"] + max_marker_gap))
    ]
    drop_times = to_drop.unique()  # type: ignore
    logger.info(
        f"Found {to_drop.shape[0]} rows (forming {len(drop_times)} contiguous gaps) where "
        f"the time since the last marker datapoint is greater than the maximum expected gap: {max_marker_gap:.4f}"
    )

    df = cast(pd.DataFrame, df[~asof["time"].isin(drop_times)])  # type: ignore

    num_dropped = asof.shape[0] - df.shape[0]
    logger.info(f"Dropped {num_dropped} out of {df.shape[0]} rows")

    df.set_index("time", inplace=True)
    df[marker_cols] = df[marker_cols].interpolate(method="slinear")
    df = df.dropna(subset=marker_cols)
    df.reset_index(inplace=True)

    df = df.drop(
        columns=[
            "left_pupil",
            "right_pupil",
            "input",
        ]
    )

    return df


def smooth_data(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    eyelink_freq: float,
    markers_freq: float,
    eyelink_window: float | None,
    markers_window: float | None,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Smooths the data by applying a rolling window to the data.
    """
    eyelink_df = eyelink_df.copy(deep=True)
    markers_df = markers_df.copy(deep=True)

    if eyelink_window is not None:
        eyelink_df = reindex_steady_time(eyelink_df, eyelink_freq, on="time")
        window_length = int(eyelink_window * eyelink_freq)
        for col in ["left_x", "left_y", "right_x", "right_y"]:
            spline = savgol_filter(
                eyelink_df[col],
                window_length=window_length,
                polyorder=3,
            )
            eyelink_df[col] = spline

    if markers_window is not None:
        markers_df = reindex_steady_time(markers_df, markers_freq, on="time")
        window_length = int(markers_window * markers_freq)
        for col in ["marker_x", "marker_y", "marker_z"]:
            markers_df[col] = savgol_filter(
                markers_df[col],
                window_length=window_length,
                polyorder=3,
            )

    return eyelink_df, markers_df


def preprocess_data(
    session_dir: os.PathLike,
    config: Mapping[str, Any],
    *,
    marker_idx: int,
    start_time: float = 0.0,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """
    Preprocesses the data by cleaning and merging the eye tracker and optical marker data.
    """
    eyelink_path = os.path.join(session_dir, "eyelink_sample.csv")
    markers_path = os.path.join(session_dir, "markers.csv")

    eyelink_df = pd.read_csv(eyelink_path, index_col=False)
    markers_df = pd.read_csv(markers_path, index_col=False)

    eyelink_freq = config["preprocess"]["eyelink_freq"]
    markers_freq = config["preprocess"]["markers_freq"]

    logger.info("Formatting columns")
    raw_eyelink_df, raw_markers_df = format_columns(
        eyelink_df,
        markers_df,
        eyelink_freq,
        markers_freq,
        marker_idx=marker_idx,
    )
    logger.info("Cleaning data")
    eyelink_df, markers_df = clean_data(
        raw_eyelink_df, raw_markers_df, start_time
    )
    # logger.info("Smoothing data")
    # eyelink_df, markers_df = smooth_data(
    #     eyelink_df,
    #     markers_df,
    #     eyelink_freq,
    #     markers_freq,
    #     eyelink_window=config["preprocess"]["eyelink_filter_window"],
    #     markers_window=config["preprocess"]["markers_filter_window"],
    # )
    logger.info("Merging eyelink and markers")
    df = merge_eyelink_markers(eyelink_df, markers_df, eyelink_freq)
    logger.info("Interpolating markers")
    df = interpolate_markers(df, config["preprocess"]["max_marker_gap"])

    path = os.path.join(session_dir, config["preprocess"]["filename"])
    df.to_csv(path, index=False)
    logger.info(f"Saved data to {path}")

    return df, raw_eyelink_df, raw_markers_df


def main(args=None):
    import argparse

    import yaml

    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s - %(message)s"
    )

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
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default=os.path.join(
            os.environ["TABLETOP_DIR"], "config", "gaze_calibration.yaml"
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
        "--visualize",
        action="store_true",
        help="Visualize the data.",
    )
    args = parser.parse_args(args)

    with open(args.config, "r") as f:
        config = cast(Mapping[str, Any], yaml.safe_load(f))

    df, raw_eyelink_df, raw_markers_df = preprocess_data(
        args.session_dir,
        config,
        marker_idx=args.marker_idx,
        start_time=args.start_time,
    )

    if args.visualize:
        from tabletop_py.gaze.visualize import plot_eyelink_markers

        logger.info("Visualizing data")
        eyelink_freq = config["preprocess"]["eyelink_freq"]
        markers_freq = config["preprocess"]["markers_freq"]

        plot_eyelink_markers(
            raw_eyelink_df,
            freq=eyelink_freq,
            markers_df=raw_markers_df,
            markers_freq=markers_freq,
            title="Raw data",
            save_path=os.path.join(args.session_dir, "raw.png"),
        )

        plot_eyelink_markers(
            df,
            freq=eyelink_freq,
            title="Preprocessed data",
            save_path=os.path.join(args.session_dir, "preprocessed.png"),
        )


if __name__ == "__main__":
    main()
