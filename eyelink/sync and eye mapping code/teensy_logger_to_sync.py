import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def process_teensy(file_path, output_path):
    """
    Process a Teensy log file to normalize timestamp data and ensure continuity in input values.
    The function also outputs the processed data to a CSV file.

    Parameters:
        file_path (str): The path to the CSV file containing timestamp and input data.
        output_path (str): The path where the processed data CSV file will be saved.

    Returns:
        new_df (DataFrame): A DataFrame with two columns: 'Time' and 'Input'.
            'Time' is a continuous range from 0 to the maximum timestamp found in the data,
            incremented by 1 millisecond, with all timestamps rounded to three decimal places.
            'Input' is updated based on changes recorded in the input file.
    """
    # Read the CSV file, skipping the first row if it's headers or irrelevant data
    df = pd.read_csv(file_path, names=["Timestamp", "State"], skiprows=1)

    # Normalize timestamps: subtract the first timestamp to start at zero, convert to seconds,
    # and round to three decimal places for millisecond precision
    df["Timestamp"] = np.round(
        (df["Timestamp"] - df["Timestamp"].iloc[0]) / 1000, 3
    )

    # Calculate the total duration from the adjusted timestamps
    duration = df["Timestamp"].iloc[-1]

    # Generate a time range from 0 to the maximum timestamp, incremented by 1 millisecond,
    # and rounded to maintain three-decimal precision
    milliseconds = np.arange(0, duration + 0.001, 0.001)
    milliseconds = np.round(milliseconds, 3)

    # Create a new DataFrame to maintain continuous time and input data
    new_df = pd.DataFrame(
        {
            "time": milliseconds,
            "input": [df["State"].iloc[0]] * len(milliseconds),
        }
    )

    # Initialize the 'Input' column with the initial input value from the data
    new_df["input"] = df["State"].iloc[0]

    # Update the 'Input' at each timestamp where it changes in the original data
    for _, row in df.iterrows():
        timestamp = row["Timestamp"]
        input_value = row["State"]
        new_df.loc[new_df["time"] >= timestamp, "input"] = input_value

    # Output the DataFrame to a CSV file
    new_df.to_csv(output_path, index=False)

    return new_df


# Example usage:
file_path = "/Volumes/USB DISK/5_17_t2.csv"
output_path = "/Users/jack/Downloads/5_17_t2/raw/5_17_t2_teensy.csv"
teensy_df = process_teensy(file_path, output_path)
print(teensy_df)
