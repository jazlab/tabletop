"""Temporal synchronization between multiple data streams.

This module provides tools for aligning timestamps between different
data sources (e.g., EyeLink, OptiTrack, Arduino) by detecting matching
patterns in synchronized trigger signals.

The alignment process:
1. Extract time intervals between trigger pulses in each stream
2. Find matching sequences of intervals across streams
3. Compute linear regression between timestamps
4. Apply offset correction to align secondary stream to primary

Constants:
    THRESHOLD: Maximum allowed difference between intervals for matching.
    CONSECUTIVE_MATCHES: Required consecutive matches to confirm alignment.

Functions:
    find_streak_intervals: Extract time intervals from trigger signal.
    find_matching_index_and_start_time: Find alignment between streams.
    calculate_time_intervals_and_correlation: Compute sync parameters.
    align_secondary_csv: Apply alignment offset to secondary CSV.

Example:
    slope, intercept, rms_error, outliers = calculate_time_intervals_and_correlation(
        "primary.csv", "secondary.csv"
    )
    align_secondary_csv("secondary.csv", "aligned.csv", intercept)
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error

#: Maximum time difference (seconds) for interval matching
THRESHOLD = 0.04

#: Required number of consecutive interval matches for alignment
CONSECUTIVE_MATCHES = 80


def find_streak_intervals(df):
    """Extract pulse intervals from trigger signal dataframe.

    Identifies contiguous high (input=1) and low (input=0) regions,
    extracts time intervals between consecutive pulses, and returns
    the timing data.

    Args:
        df: DataFrame with "time" and "input" columns where input
            indicates pulse state (1=high, 0=low).

    Returns:
        Tuple of:
        - intervals: List of time gaps between consecutive pulses.
        - start_times: List of pulse start times (excluding first and
            last pulses if > 2 exist).
    """
    streaks = []
    current_streak = False
    for _, row in df.iterrows():
        if row["input"] == 1 and not current_streak:
            current_streak = True
            streak_start = row["time"]
        elif row["input"] == 0 and current_streak:
            current_streak = False
            streak_end = row["time"]
            streaks.append((streak_start, streak_end))
    if current_streak:
        streak_end = df.iloc[-1]["time"]
        streaks.append((streak_start, streak_end))

    if len(streaks) > 2:
        streaks = streaks[1:-1]

    intervals = [
        streaks[i][0] - streaks[i - 1][1] for i in range(1, len(streaks))
    ]
    start_times = [streak[0] for streak in streaks[1:]]
    return intervals, start_times


def find_matching_index_and_start_time(
    primary_intervals, primary_start_times, secondary_intervals
):
    """
    Finds the matching index and start time between primary and secondary intervals.

    Args:
        primary_intervals (list): List of primary intervals.
        primary_start_times (list): List of primary start times.
        secondary_intervals (list): List of secondary intervals.

    Returns:
        tuple: A tuple containing:
            - matching_index: Index of the matching intervals.
            - matching_start_time: Start time of the matching intervals.
    """
    for i in range(len(primary_intervals) - CONSECUTIVE_MATCHES):
        for j in range(len(secondary_intervals) - CONSECUTIVE_MATCHES):
            match = True
            for k in range(CONSECUTIVE_MATCHES):
                if (
                    abs(primary_intervals[i + k] - secondary_intervals[j + k])
                    > THRESHOLD
                ):
                    match = False
                    break
            if match:
                return i, primary_start_times[i]
    return None, None


def calculate_time_intervals_and_correlation(primary_csv, secondary_csv):
    """Compute synchronization parameters between two data streams.

    Extracts pulse timing intervals from both streams, finds matching
    pulse sequences, fits a linear regression between timestamps, and
    detects outliers. Visualizes results with matplotlib.

    Args:
        primary_csv: Path to primary CSV (reference timeline) with
            "time" and "input" columns.
        secondary_csv: Path to secondary CSV (to be aligned) with same
            column structure.

    Returns:
        Tuple of:
        - slope: Linear fit slope (temporal scaling factor).
        - intercept: Linear fit intercept (time offset in seconds).
        - rms_error: Root mean squared error of regression.
        - num_outliers: Count of detected outliers (2-sigma from mean).

    Notes:
        - Displays scatter plot with regression line and outliers
        - Returns (None, None, None, None) if no matching sequences
            found between streams
        - Outliers computed via residual analysis
    """
    # Load CSV files
    primary_df = pd.read_csv(primary_csv)
    secondary_df = pd.read_csv(secondary_csv)

    primary_intervals, primary_start_times = find_streak_intervals(primary_df)
    secondary_intervals, secondary_start_times = find_streak_intervals(
        secondary_df
    )

    matching_index, _ = find_matching_index_and_start_time(
        primary_intervals, primary_start_times, secondary_intervals
    )
    if matching_index is not None:
        comparison_length = min(
            len(primary_start_times[matching_index:]),
            len(secondary_start_times),
        )
        matched_primary_times = primary_start_times[
            matching_index : matching_index + comparison_length
        ]
        matched_secondary_times = secondary_start_times[:comparison_length]

        X = np.array(matched_secondary_times).reshape(-1, 1)
        y = np.array(matched_primary_times)

        model = LinearRegression()
        model.fit(X, y)
        predictions = model.predict(X)
        residuals = y - predictions
        rms_error = np.sqrt(mean_squared_error(y, predictions))

        # Calculate RMS error for the first 50 data points
        rms_error_first_50 = np.sqrt(
            mean_squared_error(y[:50], predictions[:50])
        )

        mean_res = np.mean(residuals)
        std_res = np.std(residuals)
        outlier_mask = (residuals > mean_res + 2 * std_res) | (
            residuals < mean_res - 2 * std_res
        )
        outliers = y[outlier_mask]
        non_outliers = y[~outlier_mask]

        plt.figure(figsize=(12, 6))
        plt.scatter(
            X[~outlier_mask], non_outliers, color="blue", label="Data Points"
        )
        plt.scatter(X[outlier_mask], outliers, color="red", label="Outliers")
        plt.plot(X, predictions, color="green", label="Regression Line")
        plt.title("Linear Regression with Outlier Detection")
        plt.xlabel("Secondary dataset timestamps (seconds)")
        plt.ylabel("Primary dataset timestamps (seconds)")
        plt.legend()

        stats_text = f"Slope: {model.coef_[0]:.3f}\nIntercept: {model.intercept_:.3f}\nRMS Error: {rms_error:.3f}\nRMS Error (First 50): {rms_error_first_50:.3f}\nOutliers: {len(outliers)}"
        plt.annotate(
            stats_text,
            xy=(0.95, 0.05),
            xycoords="axes fraction",
            verticalalignment="bottom",
            horizontalalignment="right",
            bbox=dict(boxstyle="round,pad=0.5", fc="white"),
        )

        plt.show()

        return model.coef_[0], model.intercept_, rms_error, len(outliers)

    else:
        print(
            "Error: No matching index found between the datasets. Please check the threshold settings or data alignment."
        )
        return None, None, None, None


def align_secondary_csv(secondary_file, aligned_secondary_file, y_intercept):
    """Apply time offset to secondary CSV timestamps.

    Adds the computed intercept (offset) to all timestamps and saves
    the result with time values rounded to 3 decimal places.

    Args:
        secondary_file: Input CSV path with "time" column.
        aligned_secondary_file: Output CSV path for aligned data.
        y_intercept: Time offset (seconds) from regression intercept
            to add to all timestamps.
    """
    secondary_df = pd.read_csv(secondary_file)
    secondary_df["time"] += y_intercept
    secondary_df["time"] = secondary_df["time"].apply(lambda x: round(x, 3))
    secondary_df.to_csv(aligned_secondary_file, index=False)
    print(f"Aligned secondary CSV file saved at: {aligned_secondary_file}")


# Example usage
primary_file = "/Users/jack/Downloads/5_21_24_t1/Raw/5_21_24_t1_teensy.csv"
secondary_file = (
    "/Users/jack/Downloads/5_21_24_t1/Raw/5_21_24_t1_optitrack.csv"
)
aligned_secondary_file = "/Users/jack/Downloads/5_21_24_t1/Processed/5_21_24_t1_optitrack_aligned.csv"

slope, intercept, rms_error, num_outliers = (
    calculate_time_intervals_and_correlation(primary_file, secondary_file)
)

if intercept is not None:
    align_secondary_csv(secondary_file, aligned_secondary_file, intercept)
else:
    print(
        "Error: Unable to align the secondary CSV file due to missing y-intercept."
    )
