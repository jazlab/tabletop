import argparse
import io
import os
import subprocess

import numpy as np
import pandas as pd


def edf_to_asc(edf_path, args=["-s", "-input", "-nflags", "-y"]) -> str:
    """Convert the EDF file to ASC format.

    Args:
        edf_path (str): The path to the EDF file.
        args (list): The arguments to pass to the edf2asc command.

    Returns:
        str: The path to the output ASC file.
    """
    try:
        subprocess.run(["edf2asc", *args, edf_path], check=False)
        return os.path.splitext(edf_path)[0] + ".asc"
    except subprocess.CalledProcessError as e:
        raise RuntimeError("Error converting EDF to ASC") from e


def asc_to_df(asc_path):
    """
    Read an EyeLink ASC file and return a DataFrame.

    Args:
        eyelink_file_path (str): The path to the EyeLink ASC file.

    Returns:
        DataFrame: The DataFrame containing the processed data.
    """

    print(f"Reading file {asc_path}...")
    with open(asc_path, "r") as f:
        file_text = f.read().splitlines()

    print("Creating DataFrame")
    df = pd.read_csv(
        io.StringIO("\n".join(file_text)),
        delim_whitespace=True,
        index_col=False,
        na_values=["."],
        on_bad_lines="warn",
    )

    if df.shape[1] == 5:
        column_names = ["time", "left_x", "left_y", "left_pupil", "input"]
    elif df.shape[1] == 8:
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
    else:
        raise ValueError("Unknown file format")

    df.columns = column_names

    # Check if time is monotonically increasing
    if not np.all(np.diff(df["time"]) >= 0):
        non_monotonic_indices = np.where(np.diff(df["time"]) < 0)[0]
        raise RuntimeError(
            f"Time is not monotonically increasing at "
            f"{len(non_monotonic_indices)} indices: "
            f"{str(non_monotonic_indices[:5].tolist())[1:-1]}, ..."
        )
    else:
        print("Time is monotonically increasing ✓")

    # Start time at 0 and convert to seconds
    df["time"] = (df["time"] - df["time"][0]) / 1000.0
    time = df["time"]
    print(f"Start time: {np.min(time)}")
    print(f"End time: {np.max(time)}")

    return df


def replace_input(df):
    """
    Modify the DataFrame by replacing values in the 'input' column and selecting specific columns.

    Args:
        df (DataFrame): The input DataFrame.

    Returns:
        DataFrame: The modified DataFrame.
    """
    df["input"] = df["input"].replace({255: 1, 247: 0})
    return df


def edf_to_csv(edf_path, output_path=None):
    """
    Convert an EDF file to a CSV file.

    Args:
        input_file (str): The path to the input ASC file.
        output_file (str): The path to the output CSV file.
    """
    asc_path = edf_to_asc(edf_path)
    df = asc_to_df(asc_path)
    df = replace_input(df)
    if output_path is None:
        output_path = os.path.splitext(edf_path)[0] + ".csv"
    df.to_csv(output_path, index=False)
    print(f"Converted {edf_path} to {output_path}")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Convert an EDF file to a CSV file."
    )
    parser.add_argument(
        "edf_file", type=str, help="The path to the input EDF file."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="The path to the output CSV file.",
    )
    args = parser.parse_args(args)

    edf_to_csv(args.edf_file, args.output)


if __name__ == "__main__":
    main()
