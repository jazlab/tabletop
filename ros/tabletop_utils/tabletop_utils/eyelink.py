import argparse
import io
import os
import subprocess
from typing import Iterable, Optional

import numpy as np
import pandas as pd


def edf_to_asc(
    path: str, cli_args: Iterable[str], output_dir: Optional[str] = None
) -> str:
    """Convert the EDF file to ASC format.

    Args:
        path: The path to the EDF file.
        args: The arguments to pass to the edf2asc command.
        output_dir: The directory to save the output ASC file. If not provided,
            the output will be saved in the same directory as the EDF file with
            the same basename and .asc extension.

    Returns:
        str: The path to the output ASC file.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"File {path} does not exist")
    if os.path.isdir(path):
        raise IsADirectoryError(f"File {path} is a directory")

    cmd = ["edf2asc", *cli_args]
    if output_dir is not None:
        if "-p" in cli_args:
            raise ValueError("Cannot use -p and output_dir together")
        os.makedirs(output_dir, exist_ok=True)
        cmd.extend(["-p", output_dir])
    else:
        output_dir = os.path.dirname(path)
    cmd.append(path)

    result = subprocess.run(cmd, check=False, capture_output=True, text=True)

    if "converted successfully" not in result.stdout.lower():
        raise RuntimeError(f"Failed to convert EDF to ASC: {result.stdout}")

    basename = os.path.splitext(os.path.basename(path))[0]
    return os.path.join(output_dir, f"{basename}.asc")


def asc_to_df(
    path: str, input_mapping: Optional[dict[int, int]] = None
) -> pd.DataFrame:
    """
    Read an EyeLink ASC file and return a DataFrame.

    Args:
        path (str): The path to the EyeLink ASC file.
        input_mapping (dict): A dictionary mapping input values to new values.
            The default mapping is {255: 1, 247: 0}.

    Returns:
        DataFrame: The DataFrame containing the processed data.
    """

    with open(path, "r") as f:
        file_text = f.read().splitlines()

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

    # Start time at 0 and convert to seconds
    df["time"] = (df["time"] - df["time"][0]) / 1000.0

    # Replace input values
    if input_mapping is not None:
        df["input"] = df["input"].replace(input_mapping)

    return df


def edf_to_csv(
    edf_path: str,
    *,
    edf2asc_args: Iterable[str] = ("-s", "-input", "-nflags", "-y"),
    input_mapping: Optional[dict[int, int]] = None,
    output_path: Optional[str] = None,
    keep_asc: bool = False,
) -> str:
    """
    Convert an EDF file to a CSV file.

    Args:
        input_file (str): The path to the input ASC file.
        output_file (str): The path to the output CSV file.
    """
    output_dir = None
    if output_path is None:
        output_path = os.path.splitext(edf_path)[0] + ".csv"
    else:
        output_dir = os.path.dirname(output_path)

    asc_path = edf_to_asc(edf_path, edf2asc_args, output_dir=output_dir)
    df = asc_to_df(asc_path, input_mapping)
    df.to_csv(output_path, index=False)
    if not keep_asc:
        os.remove(asc_path)

    return output_path


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Convert an EDF file to a CSV file."
    )
    parser.add_argument(
        "edf_path", type=str, help="The path to the input EDF file."
    )
    parser.add_argument(
        "--edf2asc-args",
        type=str,
        default="-s -input -nflags -y",
        help="The arguments to pass to the edf2asc command.",
    )
    parser.add_argument(
        "--input-mapping",
        type=str,
        nargs="+",
        default=["255:1", "247:0"],
        help="The input mapping to pass to the asc_to_df command. Format: 'key:value'.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="The path to the output CSV file. If not provided, the output will be saved in the same directory as the EDF file.",
    )
    parser.add_argument(
        "--keep-asc",
        action="store_true",
        help="Keep the ASC file after conversion. If not provided, the ASC file will be deleted.",
    )
    args = parser.parse_args(args)

    args.edf2asc_args = args.edf2asc_args.split()

    args.input_mapping = {
        int(key.split(":")[0]): int(key.split(":")[1])
        for key in args.input_mapping
    }

    edf_to_csv(**vars(args))


if __name__ == "__main__":
    main()
