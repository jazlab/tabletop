import argparse
import io
import os

import numpy as np
import pandas as pd


def process_string(s):
    """
    Process a string and return its float value or NaN if it's a dot.

    Args:
        s (str): The string to process.

    Returns:
        float or NaN: The float value of the string or NaN if it's a dot.
    """
    if s == ".":
        return np.nan
    else:
        return float(s)


def read_eyelink_file(eyelink_file_path):
    """
    Read an EyeLink ASC file and return a DataFrame.

    Args:
        eyelink_file_path (str): The path to the EyeLink ASC file.

    Returns:
        DataFrame: The DataFrame containing the processed data.
    """
    column_names = [
        "time",
        "left_x",
        "left_y",
        "left_pupil",
        "right_x",
        "right_y",
        "right_pupil",
        "input",
    ]

    print(f"Reading file {eyelink_file_path}...")
    with open(eyelink_file_path, "r") as f:
        file_text = f.read().splitlines()

    # Filter out empty lines and non-numeric lines
    file_text = [
        line
        for line in file_text
        if line.strip() and line.split()[0].isdigit()
    ]

    print("Creating DataFrame")
    dataframe = pd.read_csv(
        io.StringIO("\n".join(file_text)),
        delim_whitespace=True,
        index_col=False,
        names=column_names,
        usecols=range(8),
    )

    for column in dataframe.columns:
        dataframe[column] = [
            process_string(x) for x in dataframe[column].tolist()
        ]

    start_time = dataframe["time"].tolist()[0]
    dataframe["time"] = (dataframe["time"] - start_time) / 1000.0
    time = dataframe["time"]
    print(f"Start time: {np.min(time)}")
    print(f"End time: {np.max(time)}")

    return dataframe


def modify_dataframe(df):
    """
    Modify the DataFrame by replacing values in the 'input' column and selecting specific columns.

    Args:
        df (DataFrame): The input DataFrame.

    Returns:
        DataFrame: The modified DataFrame.
    """
    df.iloc[:, 7] = df.iloc[:, 7].replace({255: 1, 247: 0})
    # df = df.iloc[:, [0, 7]] uncomment this if you only want the input column and timestamps to be included
    return df


def convert_asc_to_csv(input_file, output_file):
    """
    Convert an ASC file to a CSV file.

    Args:
        input_file (str): The path to the input ASC file.
        output_file (str): The path to the output CSV file.
    """
    eyelink_df = read_eyelink_file(input_file)
    modified_df = modify_dataframe(eyelink_df)
    modified_df.to_csv(output_file, index=False)
    print(f"Converted {input_file} to {output_file}")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Convert an ASC file to a CSV file."
    )
    parser.add_argument(
        "input_file", type=str, help="The path to the input ASC file."
    )
    parser.add_argument(
        "-o",
        "--output-file",
        type=str,
        default=None,
        help="The path to the output CSV file.",
    )
    args = parser.parse_args(args)

    if args.output_file is None:
        args.output_file = os.path.splitext(args.input_file)[0] + ".csv"

    convert_asc_to_csv(args.input_file, args.output_file)


if __name__ == "__main__":
    main()
