import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error

# Constants
THRESHOLD = 0.04
CONSECUTIVE_MATCHES = 80


def find_streak_intervals(df):
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
    """
    Calculates time intervals and correlation between primary and secondary CSV files.

    Args:
        primary_csv (str): Path to the primary CSV file.
        secondary_csv (str): Path to the secondary CSV file.

    Returns:
        tuple: A tuple containing:
            - slope: Slope of the regression line.
            - intercept: Intercept of the regression line.
            - rms_error: Root mean squared error of the regression.
            - num_outliers: Number of outliers detected.
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
    """
    Aligns the secondary CSV file by adding the y-intercept to the timestamps and saves the aligned file.

    Args:
        secondary_file (str): Path to the secondary CSV file.
        aligned_secondary_file (str): Path to save the aligned secondary CSV file.
        y_intercept (float): Y-intercept value to be added to the timestamps.
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
