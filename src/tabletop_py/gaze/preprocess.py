import logging
import os
from collections.abc import Mapping
from typing import Any, Optional, cast

import numpy as np
import pandas as pd
from pylink.constants import MISSING_DATA
from scipy.signal import savgol_filter
from scipy.stats import zscore

logger = logging.getLogger(__name__)

pd.options.mode.copy_on_write = True


EYELINK_POS_COLS = ["left_x", "left_y", "right_x", "right_y"]
EYELINK_DATA_COLS = EYELINK_POS_COLS + ["left_pupil", "right_pupil"]

MARKER_DATA_COLS = ["marker_x", "marker_y", "marker_z"]


def reindex_and_interpolate(
    df: pd.DataFrame,
    new_idx: np.ndarray | pd.Series,
    on: str,
    *,
    direction: str = "backward",
    tolerance: Optional[float] = None,
) -> pd.DataFrame:
    """
    Reindexes and interpolates the data onto a new time grid while maintaining
    the temporal gaps in the data.

    Args:
        df: The dataframe to reindex.
        new_idx: The new time grid to reindex onto.
        on: The column to reindex on.
        direction: The direction to reindex in.
        tolerance: The temporal tolerance for the interpolation. Sets the
            maximum allowed time difference between the new and old time
            points. Any new time point further than the tolerance from the
            closest old time point is set to NaN.

    Returns:
        The reindexed and interpolated dataframe.
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


def reindex_and_interpolate_steady_time(
    df: pd.DataFrame,
    *,
    freq: float,
    on: str,
    direction: str = "backward",
    tolerance: Optional[float] = None,
) -> pd.DataFrame:
    """
    Interpolates the data onto a steady time grid while maintaining the temporal gaps in the data.

    Args:
        df: The dataframe to reindex.
        freq: The frequency of the data.
        on: The column to reindex on.
        direction: The direction to reindex in.
        tolerance: The temporal tolerance for the interpolation. Sets the
            maximum allowed time difference between the new and old time
            points. Any new time point further than the tolerance from the
            closest old time point is set to NaN. If not provided, a default
            value of (1 + 1e-3) / freq is used.

    Returns:
        The reindexed and interpolated dataframe.
    """
    steady_idx = np.arange(df[on].min(), df[on].max(), 1 / freq)
    return reindex_and_interpolate(
        df, steady_idx, on=on, direction=direction, tolerance=tolerance
    )


def smooth_rolling(
    df: pd.DataFrame,
    *,
    columns: list[str],
    on: str,
    freq: float,
    window: float,
    on_unit: str = "s",
):
    df = df.copy()

    if df[on].dtype == float:
        df["datetime"] = pd.to_datetime(df[on], unit=on_unit)

    td = pd.to_timedelta(window, unit=on_unit)  # type: ignore
    rolling = df.rolling(
        on="datetime",
        center=True,
        window=td,
        min_periods=int(window * freq) - 1,
    )

    df[columns] = rolling[columns].mean().drop(columns=["datetime"])

    df = df.drop(columns=["datetime"])
    return df


def smooth_savgol(
    df: pd.DataFrame,
    *,
    columns: list[str],
    freq: float,
    on: str,
    window: float,
    polyorder: int = 3,
    deriv: int = 0,
    reindex_tolerance: Optional[float] = None,
):
    """
    Smooths the data by reindexing to steady time and applying a Savitzky-Golay filter.

    Args:
        df: The data to smooth.
        columns: The columns to smooth.
        freq: The frequency of the data.
        on: The column to smooth on.
        window: The window size for the Savitzky-Golay filter.
        polyorder: The polynomial order for the Savitzky-Golay filter.
        deriv: The order of the derivative to compute.
        reindex_tolerance: The temporal tolerance for the interpolation. Sets the
            maximum allowed time difference between the new and old time
            points. Any new time point further than the tolerance from the
            closest old time point is set to NaN. If not provided, a default
            value of (1 + 1e-3) / freq is used.

    Returns:
        The smoothed data.
    """
    df = df.copy()

    if reindex_tolerance is None:
        reindex_tolerance = 3 / freq

    df = reindex_and_interpolate_steady_time(
        df, freq=freq, on=on, tolerance=reindex_tolerance
    )

    if deriv > 0:
        delta = 1 / freq
    else:
        delta = 1.0

    window_length = int(window * freq)
    if window_length > df.shape[0]:
        raise ValueError(
            f"Window length {window_length} is greater than the number of rows {df.shape[0]}"
        )
    for col in columns:
        df[col] = savgol_filter(
            df[col],
            window_length=window_length,
            polyorder=polyorder,
            deriv=deriv,
            delta=delta,
        )

    return df


def calculate_eyelink_speed(df: pd.DataFrame) -> pd.DataFrame:
    """
    Calculates the speed of the eye data.
    """
    df = df.copy()
    df["left_speed"] = np.linalg.norm(
        np.gradient(df[["left_x", "left_y"]], df["time"], axis=0), axis=1
    )
    df["right_speed"] = np.linalg.norm(
        np.gradient(df[["right_x", "right_y"]], df["time"], axis=0), axis=1
    )
    return df


def verify_timestamps(df: pd.DataFrame, freq: float, freq_rtol: float = 1e-3):
    """
    Verifies the integrity of the timestamps.

    Args:
        df: The data to verify.
        freq: The expected frequency of the data.
        freq_rtol: The relative tolerance for the frequency.
    """
    # Check that the frame number and times are monotonic increasing
    for col in df.columns:
        if not df[col].is_monotonic_increasing:
            raise ValueError(f"{col} is not monotonically increasing")
        diff_mean = df[col].diff().mean()
        if not np.isclose(diff_mean, 1 / freq, rtol=freq_rtol):
            raise ValueError(
                f"{col} diff mean of {diff_mean:.6f} is not close to the expected value {1 / freq:.6f}"
            )
        diff_std = df[col].diff().std()
        if not np.isclose(diff_std, 0, atol=diff_mean * 0.5):
            raise ValueError(
                f"{col} diff std of {diff_std:.6f} is not close to 0"
            )


def format_eyelink_columns(
    df: pd.DataFrame,
    *,
    freq: float,
    verify: bool = True,
    freq_rtol: float = 1e-3,
) -> pd.DataFrame:
    """
    Formats the timestamps to seconds and checks for monotonicity.

    Args:
        df: The eye tracker data.
        freq: The expected frequency of the data.
        verify: Whether to verify the timestamps and monotonicity.
        freq_rtol: The relative tolerance for the frequency.

    Returns:
        The formatted eye tracker data.
    """
    df = df.copy()

    # Convert ROS timestamp to seconds
    df["time"] = df["header.stamp.sec"] + df["header.stamp.nanosec"] / 1e9

    if verify:
        # Convert timestamps to seconds
        df["bag_time"] = df.bag_time_ns / 1e9
        df["eyelink_time"] = df["eyelink_time_ms"] / 1e3

        # Verify the timestamps
        verify_timestamps(
            df[["time", "bag_time", "eyelink_time"]],  # type: ignore
            freq,
            freq_rtol=freq_rtol,
        )

    df = cast(
        pd.DataFrame,
        df[["time", *EYELINK_DATA_COLS]].astype(float),
    )

    return df


def format_marker_columns(
    df: pd.DataFrame,
    *,
    marker_idx: int,
    freq: float,
    verify: bool = True,
    freq_rtol: float = 1e-3,
    max_marker_time_correction: float = 0.01,
) -> pd.DataFrame:
    """
    Formats the timestamps to seconds and checks for monotonicity.

    Args:
        df: The optical marker data.
        marker_idx: The index of the marker to format.
        freq: The expected frequency of the optical marker data.
        verify: Whether to verify the timestamps and monotonicity.
        freq_rtol: The relative tolerance for the frequency.
        max_marker_time_correction: The maximum allowed time correction for the optical marker data.

    Returns:
        The formatted optical marker data.
    """
    df = df.copy()

    # Rename columns to the expected format
    if f"markers[{marker_idx}].id_type" not in df.columns:
        raise ValueError(
            f"Marker {marker_idx} not found. Columns: {df.columns}"
        )
    df[MARKER_DATA_COLS] = df[
        [
            f"markers[{marker_idx}].translation.x",
            f"markers[{marker_idx}].translation.y",
            f"markers[{marker_idx}].translation.z",
        ]
    ]

    # Convert ROS timestamp to seconds
    df["time"] = df["header.stamp.sec"] + df["header.stamp.nanosec"] / 1e9

    if verify:
        # Verify that the frame number is monotonically increasing
        if not df["frame_number"].is_monotonic_increasing:
            raise ValueError("Frame number is not monotonically increasing")

        # Convert timestamps to seconds
        df["bag_time"] = df.bag_time_ns / 1e9
        df["original_time"] = (
            df["header_original.stamp.sec"]
            + df["header_original.stamp.nanosec"] / 1e9
        )

        # Verify the timestamps
        verify_timestamps(
            df[["time", "original_time", "bag_time"]],  # type: ignore
            freq,
            freq_rtol=freq_rtol,
        )

        # Check that the marker time is within the expected range
        correction = df["original_time"] - df["time"]
        if correction.min() < 0:
            raise ValueError(
                f"Marker time correction is negative for {correction[correction < 0].shape[0]} out of {correction.shape[0]} rows, with min: {correction.min():.4f}"
            )
        if correction.max() > max_marker_time_correction:
            raise ValueError(
                f"Marker time correction is too large: {correction.max():.4f} > {max_marker_time_correction:.4f}"
            )

    # Keep only the expected columns
    df = cast(
        pd.DataFrame,
        df[["time", *MARKER_DATA_COLS]].astype(float),
    )

    return df


def standardize_timestamps(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    *,
    start_time: float = 0.0,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Standardizes the timestamps of the eye tracker and optical marker data.

    Args:
        eyelink_df: The eye tracker data.
        markers_df: The optical marker data.
        start_time: The start time of the data to select.

    Returns:
        The standardized eye tracker and optical marker data.
    """
    eyelink_df = eyelink_df.copy()
    markers_df = markers_df.copy()

    min_time = min(eyelink_df["time"].min(), markers_df["time"].min())
    eyelink_df["time"] = eyelink_df["time"] - min_time
    markers_df["time"] = markers_df["time"] - min_time

    # Keep only data after provided start time
    eyelink_df = cast(
        pd.DataFrame,
        eyelink_df[eyelink_df["time"] >= start_time],
    )
    markers_df = cast(
        pd.DataFrame,
        markers_df[markers_df["time"] >= start_time],
    )

    return eyelink_df, markers_df


def clean_eyelink_data(
    df: pd.DataFrame,
    *,
    min_eye_pos: float | None,
    max_eye_pos: float | None,
    max_zscore: float | None,
) -> pd.DataFrame:
    """
    Cleans the data by selecting the relevant columns and dropping invalid rows.

    Args:
        df: The eye tracker data.
        min_eye_pos: The minimum eye position to select.
        max_eye_pos: The maximum eye position to select.
    """
    df = df.copy()

    # Should not have any missing data to start with
    assert df.isna().to_numpy().sum() == 0

    # Remove rows with missing data
    assert (
        df[EYELINK_POS_COLS] - df[EYELINK_POS_COLS].astype(int)
    ).to_numpy().sum() == 0
    invalid_mask = (df[EYELINK_POS_COLS].astype(int) == MISSING_DATA).any(
        axis=1
    )
    logger.info(
        f"Removing {invalid_mask.sum()} out of {df.shape[0]} samples with missing data"
    )
    df = cast(pd.DataFrame, df[~invalid_mask])

    if max_zscore is not None:
        invalid_mask = (np.abs(zscore(df[EYELINK_POS_COLS])) > max_zscore).any(  # type: ignore
            axis=1
        )
        df = cast(pd.DataFrame, df[~invalid_mask])

    # Keep only data within the expected eye position range
    if min_eye_pos is not None and max_eye_pos is not None:
        invalid_mask = (
            (df[EYELINK_POS_COLS] < min_eye_pos)
            | (df[EYELINK_POS_COLS] > max_eye_pos)
        ).any(axis=1)
        logger.info(
            f"Removing {invalid_mask.sum()} out of {df.shape[0]} samples with "
            f"eye position outside the expected range ({min_eye_pos}, {max_eye_pos})"
        )
        df = cast(pd.DataFrame, df[~invalid_mask])

    return df


def clean_marker_data(df: pd.DataFrame) -> pd.DataFrame:
    """
    Cleans the data by selecting the relevant columns and dropping invalid rows.

    Args:
        df: The optical marker data.

    Returns:
        The standardized marker data.
    """
    df = df.copy()

    df = df.dropna()

    # zscores = stats.zscore(
    #     df[MARKER_DATA_COLS]
    # )
    # df = df[(np.abs(zscores) < 3.0).all(axis=1)]  # type: ignore

    return df


def merge_data(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    *,
    eyelink_freq: float,
) -> pd.DataFrame:
    """
    Merges the eyelink and marker data on closest time.
    """
    df = reindex_and_interpolate_steady_time(
        eyelink_df, freq=eyelink_freq, on="time", tolerance=3 / eyelink_freq
    )
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
    df: pd.DataFrame, *, max_marker_gap: float
) -> pd.DataFrame:
    """
    Interpolates the marker data onto the eyelink time points.
    """
    df = df.copy()

    marker_cols = [col for col in df.columns if "marker" in col]
    gaps = df["time"][df[marker_cols].notna().all(axis=1)].diff()  # type: ignore
    logger.info(
        f"Marker time gaps after merging | "
        f"min: {gaps.min():.4f}, "
        f"max: {gaps.max():.4f}"
    )

    # Remove rows where the gap between marker datapoints is too large to be interpolated accurately
    asof = cast(pd.DataFrame, df.asof(df.index, subset=marker_cols))
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
        ]
    )

    return df


def preprocess_data(
    session_dir: os.PathLike,
    config: Mapping[str, Any],
    *,
    marker_idx: int,
    start_time: float = 0.0,
    visualize: bool = False,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """
    Preprocesses the data by cleaning and merging the eye tracker and optical marker data.
    """
    eyelink_path = os.path.join(session_dir, "eyelink_sample.csv")
    markers_path = os.path.join(session_dir, "markers.csv")

    raw_eyelink_df = pd.read_csv(eyelink_path, index_col=False)
    raw_markers_df = pd.read_csv(markers_path, index_col=False)

    eyelink_freq = config["preprocess"]["eyelink_freq"]
    markers_freq = config["preprocess"]["markers_freq"]

    logger.info("Formatting columns")
    raw_eyelink_df = format_eyelink_columns(
        raw_eyelink_df,
        freq=eyelink_freq,
        freq_rtol=config["preprocess"]["eyelink_freq_rtol"],
    )
    raw_markers_df = format_marker_columns(
        raw_markers_df,
        marker_idx=marker_idx,
        freq=markers_freq,
        freq_rtol=config["preprocess"]["markers_freq_rtol"],
        max_marker_time_correction=config["preprocess"][
            "max_marker_time_correction"
        ],
    )

    logger.info("Standardizing timestamps")
    raw_eyelink_df, raw_markers_df = standardize_timestamps(
        raw_eyelink_df, raw_markers_df, start_time=start_time
    )

    raw_eyelink_df.to_csv(
        os.path.join(session_dir, "raw_eyelink.csv"), index=False
    )
    raw_markers_df.to_csv(
        os.path.join(session_dir, "raw_markers.csv"), index=False
    )

    logger.info("Cleaning data")
    eyelink_df = clean_eyelink_data(
        raw_eyelink_df,
        min_eye_pos=config["preprocess"]["min_eye_pos"],
        max_eye_pos=config["preprocess"]["max_eye_pos"],
        max_zscore=config["preprocess"]["max_zscore"],
    )
    markers_df = clean_marker_data(raw_markers_df)

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
    df = merge_data(eyelink_df, markers_df, eyelink_freq=eyelink_freq)
    logger.info("Interpolating markers")
    df = interpolate_markers(
        df, max_marker_gap=config["preprocess"]["max_marker_gap"]
    )

    path = os.path.join(session_dir, config["preprocess"]["filename"])
    df.to_csv(path, index=False)
    logger.info(f"Saved data to {path}")

    if visualize:
        from tabletop_py.gaze.visualize import plot_eyelink_markers

        logger.info("Visualizing data")

        plot_eyelink_markers(
            raw_eyelink_df,
            freq=eyelink_freq,
            markers_df=raw_markers_df,
            markers_freq=markers_freq,
            title="Raw data",
            save_path=os.path.join(session_dir, "raw.png"),
        )

        plot_eyelink_markers(
            df,
            freq=eyelink_freq,
            title="Preprocessed data",
            save_path=os.path.join(session_dir, "preprocessed.png"),
        )

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
        "--visualize",
        action="store_true",
        help="Visualize the data.",
    )
    args = parser.parse_args(args)

    with open(args.config, "r") as f:
        config = cast(Mapping[str, Any], yaml.safe_load(f))

    preprocess_data(
        args.session_dir,
        config,
        marker_idx=args.marker_idx,
        start_time=args.start_time,
        visualize=args.visualize,
    )


if __name__ == "__main__":
    main()
