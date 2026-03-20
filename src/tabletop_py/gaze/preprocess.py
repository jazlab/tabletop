"""Data preprocessing pipeline for gaze estimation calibration.

This module handles preprocessing of eye tracking (EyeLink) and motion
capture (OptiTrack) data for gaze estimation model training. It provides
functions for timestamp alignment, data cleaning, smoothing, and merging
multiple data streams.

The preprocessing pipeline:
1. Format timestamps from ROS bag exports
2. Standardize time ranges between data streams
3. Clean invalid/missing data points
4. Reindex and interpolate to common time grid
5. Apply smoothing (Savitzky-Golay filter)
6. Merge eye tracking and marker data

Constants:
    EYELINK_POS_COLS: Eye position column names for both eyes.
    EYELINK_DATA_COLS: Input features for model training.
    MARKER_DATA_COLS: Target position columns (3D marker position).

Functions:
    reindex_and_interpolate: Resample data to new time grid.
    verify_timestamps: Validate timestamp monotonicity and frequency.
    smooth_savgol: Apply Savitzky-Golay smoothing filter.
    clean_eyelink_data: Remove invalid eye tracking samples.
    clean_marker_data: Remove invalid marker samples.
    preprocess_data: Full preprocessing pipeline.
    main: CLI entry point.

Example:
    python -m tabletop_py.gaze.preprocess -d /path/to/session --visualize
"""

import logging
import os
from collections.abc import Mapping, MutableMapping
from copy import deepcopy
from typing import Any, Literal, Optional, cast

import numpy as np
import pandas as pd
import yaml
from scipy.signal import savgol_filter
from scipy.stats import zscore

try:
    from pylink.constants import MISSING_DATA
except ImportError:
    MISSING_DATA = -32768

logger = logging.getLogger(__name__)

pd.options.mode.copy_on_write = True


#: Column names for eye position data (both eyes, x and y)
EYELINK_POS_COLS = ["left_x", "left_y", "right_x", "right_y"]

#: Input feature columns for gaze estimation model
EYELINK_DATA_COLS = EYELINK_POS_COLS  # + ["left_pupil", "right_pupil"]

#: Column names for marker position data (x, y, z)
MARKER_POS_COLS = ["marker_x", "marker_y", "marker_z"]

#: Target output columns for gaze estimation model
MARKER_DATA_COLS = MARKER_POS_COLS


def verify_timestamps(
    df: pd.DataFrame, freq: float, *, freq_rtol: float, freq_var_tol: float
):
    """
    Verifies the integrity of the timestamps.

    Args:
        df: The data to verify.
        freq: The expected frequency of the data.
        freq_rtol: The relative tolerance for the frequency.
    """
    # Check that the timestamps are non-NAN and monotonically increasing
    for col in df.columns:
        if df[col].isna().any():
            raise ValueError(f"df[{col}] has NAN values")

        if not df[col].is_monotonic_increasing:
            raise ValueError(f"df[{col}] is not monotonically increasing")

        diff_mean = df[col].diff().mean()
        if not np.isclose(diff_mean, 1 / freq, rtol=freq_rtol):
            raise ValueError(
                f"{col} diff mean of {diff_mean:.6f} is not close to the expected value {1 / freq:.6f}"
            )

        diff_std = df[col].diff().std()
        if not np.isclose(diff_std, 0, atol=freq_var_tol):
            raise ValueError(
                f"{col} diff std of {diff_std:.6f} is not close to 0"
            )


def eyelink_array_to_samples(df: pd.DataFrame) -> pd.DataFrame:
    i = 0
    dfs: list[pd.DataFrame] = []
    while f"samples[{i}].header.stamp.sec" in df.columns:
        cols = filter(lambda col: f"samples[{i}]" in col, df.columns)
        new_df = df[cols]
        new_df.columns = [
            col.replace(f"samples[{i}].", "") for col in new_df.columns
        ]
        dfs.append(new_df)
        i += 1

    df = pd.concat(dfs, ignore_index=True)
    df["time"] = df["header.stamp.sec"] + df["header.stamp.nanosec"] / 1e9
    df = df.sort_values(by="time", axis=0)
    df = df.reset_index(drop=True)
    df = df.drop(columns="time")

    return df


def format_eyelink_columns(
    df: pd.DataFrame,
    *,
    freq: float,
    verify: bool = True,
    freq_rtol: float | None = None,
    freq_var_tol: float | None = None,
) -> pd.DataFrame:
    """
    Formats the timestamps to seconds and checks for monotonicity.

    Args:
        df: The eye tracker data.
        freq: The expected frequency of the data.
        verify: Whether to verify the timestamps and monotonicity.
        freq_rtol: The relative tolerance for the frequency.
        freq_var_tol: The absolute tolerance for the frequency.

    Returns:
        The formatted eye tracker data.
    """
    df = df.copy()

    # Convert ROS timestamp to seconds
    df["time"] = df["header.stamp.sec"] + df["header.stamp.nanosec"] / 1e9

    if verify:
        if freq_rtol is None or freq_var_tol is None:
            raise ValueError(
                "freq_rtol and freq_var_tol must be provided if verify is True"
            )

        time_cols = ["time", "eyelink_time"]

        # Convert timestamps to seconds
        df["eyelink_time"] = df["eyelink_time_ms"] / 1e3

        if "bag_time_ns" in df.columns:
            df["bag_time"] = df.bag_time_ns / 1e9
            time_cols.append("bag_time")

        # Verify the timestamps
        verify_timestamps(
            df[time_cols],  # type: ignore
            freq,
            freq_rtol=freq_rtol,
            freq_var_tol=freq_var_tol,
        )

    df = df[["time", *EYELINK_DATA_COLS]].astype(float)

    return df


def format_marker_columns(
    df: pd.DataFrame,
    *,
    marker_idx: int,
    freq: float,
    verify: bool = True,
    freq_rtol: Optional[float] = None,
    freq_var_tol: Optional[float] = None,
    max_marker_time_correction: Optional[float] = None,
) -> pd.DataFrame:
    """
    Formats the timestamps to seconds and checks for monotonicity.

    Args:
        df: The optical marker data.
        marker_idx: The index of the marker to format.
        freq: The expected frequency of the optical marker data.
        verify: Whether to verify the timestamps and monotonicity.
        freq_rtol: The relative tolerance for the frequency.
        freq_var_tol: The absolute tolerance for the frequency.
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
        if (
            freq_rtol is None
            or freq_var_tol is None
            or max_marker_time_correction is None
        ):
            raise ValueError(
                "freq_rtol, freq_var_tol, and max_marker_time_correction must be provided if verify is True"
            )

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
            # df[["time", "original_time", "bag_time"]],  # type: ignore
            df[["time"]],  # type: ignore
            freq,
            freq_rtol=freq_rtol,
            freq_var_tol=freq_var_tol,
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
    df = df[["time", *MARKER_DATA_COLS]].astype(float)

    return df


def standardize_timestamps(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Standardizes the timestamps of the eye tracker and optical marker data.
    This shifts the data so that the beginning of the shared time range
    is at 0.

    Args:
        eyelink_df: The eye tracker data.
        markers_df: The optical marker data.

    Returns:
        The standardized eye tracker and optical marker data.
    """
    eyelink_df = eyelink_df.copy()
    markers_df = markers_df.copy()

    # Shift the beginning of the shared
    min_time = max(eyelink_df["time"].min(), markers_df["time"].min())
    eyelink_df["time"] = eyelink_df["time"] - min_time
    markers_df["time"] = markers_df["time"] - min_time

    return eyelink_df, markers_df


def clip_timestamps(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    *,
    start_time: float = 0.0,
    end_time: float = float("inf"),
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Clip the timestamps of the eye tracker and optical marker data.

    This removes data outside the tighter of the provided and calculated
    shared time ranges.

    Args:
        eyelink_df: The eye tracker data.
        markers_df: The optical marker data.
        start_time: The start time (after standardizing) of the data to select.
        end_time: The end time (after standardizing) of the data to select.

    Returns:
        The standardized eye tracker and optical marker data.
    """
    eyelink_df = eyelink_df.copy()
    markers_df = markers_df.copy()

    # Filter out data outside the time range provided
    start_time = max(
        start_time,
        eyelink_df["time"].min(),
        markers_df["time"].min(),
    )  # type: ignore
    end_time = min(
        end_time,
        eyelink_df["time"].max(),
        markers_df["time"].max(),
    )  # type: ignore

    num_samples = eyelink_df.shape[0]
    invalid_mask = (eyelink_df["time"] < start_time) | (
        eyelink_df["time"] > end_time
    )
    eyelink_df = eyelink_df[~invalid_mask]
    logger.info(
        f"Dropped {invalid_mask.sum()} out of {num_samples} eyelink "
        f"samples with time outside the range ({start_time:.4f}, {end_time:.4f})"
    )

    num_samples = markers_df.shape[0]
    invalid_mask = (markers_df["time"] < start_time) | (
        markers_df["time"] > end_time
    )
    markers_df = cast(pd.DataFrame, markers_df[~invalid_mask])
    logger.info(
        f"Dropped {invalid_mask.sum()} out of {num_samples} marker "
        f"samples with time outside the range ({start_time:.4f}, {end_time:.4f})"
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
    assert not df.isna().any(axis=None)

    # Remove rows with missing data
    assert (
        df[EYELINK_DATA_COLS] - df[EYELINK_DATA_COLS].astype(int)
    ).to_numpy().sum() == 0

    num_samples = df.shape[0]
    invalid_mask = (df[EYELINK_DATA_COLS].astype(int) == MISSING_DATA).any(
        axis=1
    )
    df = df[~invalid_mask]
    logger.info(
        f"Dropped {invalid_mask.sum()} out of {num_samples} "
        f"eyelink samples with missing data"
    )

    # Keep only data within the expected eye position range
    if min_eye_pos is not None and max_eye_pos is not None:
        num_samples = df.shape[0]
        invalid_mask = (
            (df[EYELINK_POS_COLS] < min_eye_pos)
            | (df[EYELINK_POS_COLS] > max_eye_pos)
        ).any(axis=1)
        df = df[~invalid_mask]
        logger.info(
            f"Dropped {invalid_mask.sum()} out of {num_samples} eyelink samples with "
            f"eye position outside the expected range ({min_eye_pos}, {max_eye_pos})"
        )

    if max_zscore is not None:
        num_samples = df.shape[0]
        invalid_mask = (
            np.abs(zscore(df[EYELINK_DATA_COLS])) > max_zscore  # type: ignore
        ).any(axis=1)
        df = df[~invalid_mask]
        logger.info(
            f"Dropped {invalid_mask.sum()} out of {num_samples} eyelink "
            f"samples with z-score greater than {max_zscore}"
        )

    assert not (df[EYELINK_DATA_COLS].astype(int) == MISSING_DATA).any(
        axis=None
    )

    return df


def clean_marker_data(
    df: pd.DataFrame,
    *,
    min_pos: list[float] | None,
    max_pos: list[float] | None,
    max_zscore: float | None,
) -> pd.DataFrame:
    """
    Cleans the data by selecting the relevant columns and dropping invalid rows.

    Args:
        df: The optical marker data.

    Returns:
        The standardized marker data.
    """
    df = df.copy()

    df = df.dropna()

    # Keep only data within the expected position range
    if min_pos is None:
        min_pos = [float("-inf")] * 3
    else:
        min_pos = [x if x is not None else float("-inf") for x in min_pos]

    if max_pos is None:
        max_pos = [float("inf")] * 3
    else:
        max_pos = [x if x is not None else float("inf") for x in max_pos]

    if min_pos is not None and max_pos is not None:
        num_samples = df.shape[0]
        invalid_mask = (
            (df[MARKER_POS_COLS] < min_pos) | (df[MARKER_POS_COLS] > max_pos)
        ).any(axis=1)
        df = df[~invalid_mask]
        logger.info(
            f"Dropped {invalid_mask.sum()} out of {num_samples} samples with "
            f"marker position outside the expected range "
            f"({min_pos}, {max_pos})"
        )

    if max_zscore is not None:
        num_samples = df.shape[0]
        invalid_mask = (np.abs(zscore(df[MARKER_DATA_COLS])) > max_zscore).any(  # type: ignore
            axis=1
        )
        df = df[~invalid_mask]
        logger.info(
            f"Dropped {invalid_mask.sum()} out of {num_samples} samples with z-score greater than {max_zscore}"
        )

    assert not df.isna().any(axis=None)

    return df


def reindex_and_interpolate(
    df: pd.DataFrame,
    new_idx: np.ndarray | pd.Series,
    on: str,
    *,
    direction: Literal["backward", "forward", "nearest"] = "nearest",
    tolerance: Optional[float] = None,
) -> pd.DataFrame:
    """
    Reindexes and interpolates the data onto a new time grid while maintaining
    the temporal gaps in the data.

    Args:
        df: The dataframe to reindex.
        new_idx: The new time grid to reindex onto.
        on: The column to reindex on.
        direction: The direction to perform the merge_asof operation in.
        tolerance: The temporal tolerance for the interpolation. Sets the
            maximum allowed time difference between the new and old time
            points. For each new time point, if it further than the tolerance
            from the closest old time point, the interpolation is discarded and
            the row filled with NaN. If tolerance is not provided, all interpolated
            rows are kept.

    Returns:
        The reindexed and interpolated dataframe.
    """
    new_df = pd.merge_asof(
        pd.DataFrame({on: new_idx}),
        df,
        on=on,
        direction=direction,
        tolerance=tolerance,  # pyright: ignore[reportArgumentType]
    )
    isna = new_df.isna().any(axis=1)

    for col in df.columns:
        if col == on:
            continue
        data = np.interp(new_idx, df[on], df[col])
        data[isna] = np.nan  # pyright: ignore[reportIndexIssue]
        new_df[col] = data

    return new_df


def smooth_rolling_deprecated(
    df: pd.DataFrame,
    *,
    columns: list[str],
    on: str,
    freq: float,
    window: float,
    on_unit: Literal["D", "s", "ms", "us", "ns"] = "s",
    win_type: Optional[str] = None,
    win_kwargs: Optional[Mapping[str, Any]] = None,
):
    df = df.copy()

    if df[on].dtype == float:
        df["datetime"] = pd.to_datetime(df[on], unit=on_unit)

    td = pd.to_timedelta(window, unit=on_unit)  # type: ignore
    rolling = df[["datetime", *columns]].rolling(
        on="datetime",
        center=True,
        window=td,
        win_type=win_type,
        # min_periods=int(window * freq) - 1,
    )

    if win_kwargs is None:
        win_kwargs = {}

    df[columns] = (
        rolling[columns].mean(**win_kwargs).drop(columns=["datetime"])
    )

    df = df.drop(columns=["datetime"])
    return df


def smooth_rolling(
    df: pd.DataFrame,
    *,
    columns: list[str],
    on: str,
    freq: float,
    window: float,
    center: bool = False,
    min_periods: Optional[int] = None,
    win_type: Optional[str] = None,
    win_kwargs: Optional[Mapping[str, Any]] = None,
):
    df = df.copy()

    verify_timestamps(df[[on]], freq, freq_rtol=1e-3, freq_var_tol=1e-3)  # type: ignore

    window_length = int(window * freq)
    rolling = df.rolling(
        on=on,
        center=center,
        window=window_length,
        win_type=win_type,
        min_periods=min_periods,
    )

    if win_kwargs is None:
        win_kwargs = {}

    df[columns] = rolling[columns].mean(**win_kwargs).drop(columns=[on])

    return df


def smooth_savgol(
    df: pd.DataFrame,
    *,
    columns: list[str],
    on: str,
    freq: float,
    window: float,
    polyorder: int,
    deriv: int = 0,
):
    """
    Smooths the data by applying a Savitzky-Golay filter. Verifies that the timestamps
    have a constant time difference of 1/freq between consecutive rows before smoothing.

    Args:
        df: The data to smooth.
        on: The column to smooth on.
        columns: The columns to smooth.
        freq: The frequency of the data.
        window: The window size for the Savitzky-Golay filter.
        polyorder: The polynomial order for the Savitzky-Golay filter.
        deriv: The order of the derivative to compute.

    Returns:
        The smoothed data.
    """
    df = df.copy()

    verify_timestamps(df[[on]], freq, freq_rtol=1e-3, freq_var_tol=1e-3)  # type: ignore

    assert not df[on].isna().any()

    # if deriv > 0:
    #     delta = 1 / freq
    # else:
    #     delta = 1

    window_length = int(window * freq)
    if window_length > df.shape[0]:
        raise ValueError(
            f"Window length {window_length} is greater than the number of rows {df.shape[0]}"
        )
    for col in columns:
        df[col] = savgol_filter(  # pyright: ignore[reportCallIssue, reportArgumentType]
            df[col],
            window_length=window_length,
            polyorder=polyorder,
            deriv=deriv,
            delta=1 / freq,
        )

    return df


def smooth_eyelink_data(
    df: pd.DataFrame, *, method: Literal["savgol", "rolling"], **kwargs
) -> pd.DataFrame:
    """Smooths the eyelink data. See smooth_savgol for more details."""
    match method:
        case "savgol":
            return smooth_savgol(
                df, columns=EYELINK_DATA_COLS, on="time", deriv=0, **kwargs
            )
        case "rolling":
            return smooth_rolling(
                df, columns=EYELINK_DATA_COLS, on="time", **kwargs
            )
        case _:
            raise ValueError(f"Smoothing method {method} unsupported")


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
    return df[["left_speed", "right_speed"]]


def calculate_eyelink_speed_savgol(
    df: pd.DataFrame, *, freq: float, window: float, polyorder: int
):
    """
    Calculates the speed of the eye data.
    """
    vel_df = smooth_savgol(
        df,
        columns=EYELINK_POS_COLS,
        on="time",
        freq=freq,
        window=window,
        polyorder=polyorder,
        deriv=1,
    )
    vel_df["left_speed"] = np.linalg.norm(vel_df[["left_x", "left_y"]], axis=1)
    vel_df["right_speed"] = np.linalg.norm(
        vel_df[["right_x", "right_y"]], axis=1
    )

    return vel_df[["left_speed", "right_speed"]]


def calculate_marker_speed(df: pd.DataFrame) -> pd.Series:
    """
    Calculates the speed of the marker data.
    """
    df = df.copy()
    df["speed"] = np.linalg.norm(
        np.gradient(df[MARKER_DATA_COLS], df["time"], axis=0), axis=1
    )
    return df["speed"]


def filter_eyelink_by_speed(
    df: pd.DataFrame, min_speed: float, max_speed: float
) -> pd.DataFrame:
    speed = calculate_eyelink_speed(df)
    num_samples = df.shape[0]
    valid_mask = ((speed >= min_speed) & (speed <= max_speed)).all(axis=1)
    df = df[valid_mask]
    logger.info(
        f"Dropped {num_samples - df.shape[0]} out of {num_samples} eyelink samples with too fast or too slow speed"
    )
    return df


def filter_eyelink_by_speed_savgol(
    df: pd.DataFrame,
    min_speed: float,
    max_speed: float,
    *,
    freq: float,
    window: float,
    polyorder: int,
) -> pd.DataFrame:
    speed = calculate_eyelink_speed_savgol(
        df, freq=freq, window=window, polyorder=polyorder
    )
    num_samples = df.shape[0]
    valid_mask = ((speed >= min_speed) & (speed <= max_speed)).all(axis=1)
    df = df[valid_mask]
    logger.info(
        f"Dropped {num_samples - df.shape[0]} out of {num_samples} eyelink samples with too fast or too slow speed"
    )
    return df


def transform_marker_to_led(
    df: pd.DataFrame, rel_pos: list[float]
) -> pd.DataFrame:
    if len(rel_pos) != 3:
        raise ValueError("correction must be of length 3")

    df = df.copy()
    df[MARKER_POS_COLS] = df[MARKER_POS_COLS] + rel_pos
    return df


def merge_and_interpolate_data(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
    *,
    eyelink_freq: float,
    marker_interpolation_limit: float,
) -> pd.DataFrame:
    """
    Merges the marker data onto the closest time point of the eyelink data.
    Verifies that the eyelink timestamps have a constant time difference of
    1/freq between consecutive rows before merging.

    Args:
        eyelink_df: The eyelink data.
        markers_df: The marker data.
        freq: The frequency of the data.
        marker_interpolation_limit: The maximum allowed gap between marker
            data points to interpolate.

    Returns:
        The merged eyelink and marker data.
    """
    verify_timestamps(
        eyelink_df[["time"]],  # type: ignore
        eyelink_freq,
        freq_rtol=1e-3,
        freq_var_tol=1e-3,
    )

    num_marker_samples = markers_df.shape[0]

    markers_df = reindex_and_interpolate(
        markers_df,
        eyelink_df["time"],  # type: ignore
        on="time",
        tolerance=1 / eyelink_freq,
    )

    df = pd.merge(
        eyelink_df, markers_df, on="time", suffixes=("_eyelink", "_marker")
    )

    assert df.shape[0] == eyelink_df.shape[0], (
        f"Expected {eyelink_df.shape[0]} rows after merging, got {df.shape[0]}"
    )
    matched = df[MARKER_DATA_COLS].notna().all(axis=1).sum()  # type: ignore
    assert matched == num_marker_samples, (
        f"Expected {num_marker_samples} matched marker data points, got {matched}"
    )

    # gaps = df["time"][df[MARKER_DATA_COLS].notna().all(axis=1)].diff()  # type: ignore
    # logger.info(
    #     f"Marker time gaps after merging | "
    #     f"min: {gaps.min():.4f}, "
    #     f"max: {gaps.max():.4f}"
    # )
    # # Remove rows where the gap between marker datapoints is too large to be interpolated accurately
    # asof = df.asof(df.index)
    # to_drop = asof["time"][
    #     asof["time"].isna() | (df["time"] > (asof["time"] + max_marker_gap))
    # ]
    # drop_times = to_drop.unique()  # type: ignore
    # logger.info(
    #     f"Found {to_drop.shape[0]} rows (forming {len(drop_times)} contiguous gaps) where "
    #     f"the time since the last marker datapoint is greater than the maximum expected gap: {max_marker_gap:.4f}"
    # )

    # df = df[~asof["time"].isin(drop_times)]

    # num_dropped = asof.shape[0] - df.shape[0]
    # logger.info(f"Dropped {num_dropped} out of {df.shape[0]} rows")

    df = df.set_index("time")
    limit = int(marker_interpolation_limit * eyelink_freq)
    df[MARKER_DATA_COLS] = df[MARKER_DATA_COLS].interpolate(
        method="slinear",
        limit=limit,
        limit_direction="both",
        limit_area="inside",
    )
    df = df.reset_index()

    num_samples = df.shape[0]
    df = df.dropna(subset=MARKER_DATA_COLS)
    logger.info(
        f"Dropped {num_samples - df.shape[0]} out of {num_samples} samples with missing marker data"
    )

    num_samples = df.shape[0]
    df = df.dropna()
    logger.info(
        f"Dropped {num_samples - df.shape[0]} out of {num_samples} samples with missing data"
    )

    return df


def merge_data(
    eyelink_df: pd.DataFrame,
    markers_df: pd.DataFrame,
) -> pd.DataFrame:
    """
    Merges the marker data onto the closest time point of the eyelink data.
    Verifies that the eyelink timestamps have a constant time difference of
    1/freq between consecutive rows before merging.

    Args:
        eyelink_df: The eyelink data.
        markers_df: The marker data.
        freq: The frequency of the data.
        marker_interpolation_limit: The maximum allowed gap between marker
            data points to interpolate.

    Returns:
        The merged eyelink and marker data.
    """
    df = pd.merge(
        eyelink_df, markers_df, on="time", suffixes=("_eyelink", "_marker")
    )

    num_samples = df.shape[0]

    eyelink_na = df[EYELINK_DATA_COLS].isna().any(axis=1)
    marker_na = df[MARKER_DATA_COLS].isna().any(axis=1)

    df = df[~(eyelink_na | marker_na)]

    logger.info(
        f"Dropped {eyelink_na.sum()} eyelink and {marker_na.sum()} marker "
        f"samples (union: {(eyelink_na | marker_na).sum()}) with missing "
        f" data out of {num_samples} total samples"
    )

    return df


def preprocess_data(
    session_dir: str | os.PathLike,
    config: MutableMapping[str, Any] | os.PathLike | str,
    *,
    marker_idx: int,
    start_time: float = 0.0,
    end_time: float = float("inf"),
    skip_verify: bool = False,
    visualize: bool = False,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """
    Preprocesses the data by cleaning and merging the eye tracker and optical marker data.
    """
    if not isinstance(config, MutableMapping):
        with open(config, "r") as f:
            orig_config = cast(dict[str, Any], yaml.safe_load(f))
    else:
        orig_config = config

    config = deepcopy(orig_config)

    eyelink_config = cast(
        MutableMapping[str, Any], config["preprocess"]["eyelink"]
    )
    marker_config = cast(
        MutableMapping[str, Any], config["preprocess"]["marker"]
    )

    eyelink_path = os.path.join(session_dir, "eyelink_sample.csv")
    markers_path = os.path.join(session_dir, "markers.csv")

    if os.path.exists(eyelink_path):
        raw_eyelink_df = pd.read_csv(eyelink_path, index_col=False)
    else:
        eyelink_array_path = os.path.join(
            session_dir, "eyelink_sample_array.csv"
        )
        raw_eyelink_array_df = pd.read_csv(eyelink_array_path, index_col=False)
        raw_eyelink_df = eyelink_array_to_samples(raw_eyelink_array_df)

    raw_markers_df = pd.read_csv(markers_path, index_col=False)

    logger.info("Formatting columns")
    if skip_verify:
        eyelink_config["format_columns"]["verify"] = False
        marker_config["format_columns"]["verify"] = False

    raw_eyelink_df = format_eyelink_columns(
        raw_eyelink_df,
        freq=config["eyelink_freq"],
        **eyelink_config["format_columns"],
    )
    raw_markers_df = format_marker_columns(
        raw_markers_df,
        marker_idx=marker_idx,
        freq=config["markers_freq"],
        **marker_config["format_columns"],
    )

    logger.info("Standardizing timestamps")
    raw_eyelink_df, raw_markers_df = standardize_timestamps(
        raw_eyelink_df, raw_markers_df
    )

    raw_eyelink_path = os.path.join(session_dir, "raw_eyelink.csv")
    raw_markers_path = os.path.join(session_dir, "raw_markers.csv")
    logger.info(
        f"Saving raw data to {raw_eyelink_path} and {raw_markers_path}"
    )
    raw_eyelink_df.to_csv(raw_eyelink_path, index=False)
    raw_markers_df.to_csv(raw_markers_path, index=False)

    if visualize:
        from tabletop_py.gaze.visualize import plot_eyelink_markers

        logger.info("Visualizing raw data")

        plot_eyelink_markers(
            raw_eyelink_df,
            title="Raw data",
            freq=config["eyelink_freq"],
            markers_df=raw_markers_df,
            markers_freq=config["markers_freq"],
            save_path=os.path.join(session_dir, "raw.png"),
        )

    logger.info("Cleaning data")
    eyelink_df = clean_eyelink_data(raw_eyelink_df, **eyelink_config["clean"])
    markers_df = clean_marker_data(raw_markers_df, **marker_config["clean"])

    logger.info("Reindexing and interpolating data")
    min_time = max(eyelink_df["time"].min(), markers_df["time"].min())
    max_time = min(eyelink_df["time"].max(), markers_df["time"].max())
    steady_idx = np.arange(min_time, max_time, 1 / config["eyelink_freq"])

    eyelink_df = reindex_and_interpolate(
        eyelink_df,
        steady_idx,
        on="time",
        **eyelink_config["reindex_and_interpolate"],
    )
    markers_df = reindex_and_interpolate(
        markers_df,
        steady_idx,
        on="time",
        **marker_config["reindex_and_interpolate"],
    )

    # logger.info("Filtering eyelink data by speed")
    # eyelink_df = filter_eyelink_by_speed_savgol(
    #     eyelink_df,
    #     freq=config["eyelink_freq"],
    #     **eyelink_config["filter_by_speed"],
    # )
    #
    logger.info("Smoothing eyelink data")
    eyelink_df = smooth_eyelink_data(
        eyelink_df, freq=config["eyelink_freq"], **eyelink_config["smooth"]
    )

    logger.info("Filtering eyelink data by speed")
    eyelink_df = filter_eyelink_by_speed(
        eyelink_df, **eyelink_config["filter_by_speed"]
    )

    logger.info("Transforming marker position to LED position")
    markers_df = transform_marker_to_led(
        markers_df, **marker_config["transform_marker_to_led"]
    )

    logger.info("Clipping timestamps")
    eyelink_df, markers_df = clip_timestamps(
        eyelink_df, markers_df, start_time=start_time, end_time=end_time
    )

    logger.info("Merging eyelink and markers")
    df = merge_data(eyelink_df, markers_df)

    logger.info(f"Final number of samples: {df.shape[0]}")

    path = os.path.join(session_dir, config["preprocess"]["filename"])
    df.to_csv(path, index=False)
    logger.info(f"Saved data to {path}")

    if visualize:
        from tabletop_py.gaze.visualize import (
            animate_2d_dots,
            plot_eyelink_markers,
        )

        logger.info("Visualizing preprocessed data")

        plot_eyelink_markers(
            df,
            title="Preprocessed data",
            freq=config["eyelink_freq"],
            save_path=os.path.join(session_dir, "preprocessed.png"),
        )

        if False:
            left_eye = cast(np.ndarray, df[["left_x", "left_y"]].to_numpy())
            right_eye = cast(np.ndarray, df[["right_x", "right_y"]].to_numpy())
            animate_2d_dots(
                {"Left eye": left_eye, "Right eye": right_eye},
                freq=config["eyelink_freq"],
                **config["visualize"]["animate_2d_dots"],
                save_path=os.path.join(session_dir, "eyelink.mp4"),
            )

    return df, raw_eyelink_df, raw_markers_df


def main(args=None):
    import argparse

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
        "-s",
        "--start-time",
        type=float,
        default=0.0,
        help="The start time of the data to visualize in seconds, relative to the start of the session.",
    )
    parser.add_argument(
        "-e",
        "--end-time",
        type=float,
        default=float("inf"),
        help="The end time of the data to visualize in seconds, relative to the start of the session.",
    )
    parser.add_argument(
        "-m",
        "--marker-idx",
        type=int,
        default=0,
        help="The index of the marker to use.",
    )
    parser.add_argument(
        "--skip-verify",
        action="store_true",
        default=False,
        help="Skip timestamp consistency verification.",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        default=False,
        help="Visualize the data.",
    )
    args = parser.parse_args(args)

    preprocess_data(**vars(args))


if __name__ == "__main__":
    main()
