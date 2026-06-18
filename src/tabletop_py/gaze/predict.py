"""Inference and evaluation for trained gaze estimation models.

This module provides inference functions for making predictions with
trained gaze estimation models and computing evaluation metrics.

Functions:
    evaluate: Compute test metrics (MSE, RMSE, MAE, R2) on a dataloader.
    predict: Full prediction pipeline loading model and evaluating on test
        set from session directory.
    main: CLI entry point for prediction.

The prediction pipeline:
1. Load preprocessed data from session directory
2. Initialize test dataloader
3. Load trained model weights
4. Evaluate on test set to get predictions
5. Save targets and predictions to CSV
6. Optionally visualize predictions vs targets

Example:
    python -m tabletop_py.gaze.predict -d /path/to/session --visualize
"""

import logging
import os
from collections.abc import Mapping
from typing import Any, cast

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import yaml
from torcheval.metrics.functional import mean_squared_error, r2_score

from tabletop_py.gaze.utils import (
    configure_torch_dtype,
    init_criterion,
    init_model,
    init_test_dataloader,
    load_model_weights,
    seed_everything,
)

logger = logging.getLogger(__name__)


def evaluate(
    model: nn.Module,
    criterion: nn.Module,
    loader: torch.utils.data.DataLoader,
    device: torch.device,
) -> dict[str, float | torch.Tensor]:
    """Compute evaluation metrics on a test dataloader.

    Args:
        model: Trained neural network model.
        criterion: Loss function for evaluation.
        loader: DataLoader containing test samples (inputs, targets).
        device: Torch device (CPU or GPU) for computation.

    Returns:
        Dictionary containing:
        - "targets": Ground truth 3D positions (shape: [N, 3]).
        - "preds": Model predictions (shape: [N, 3]).
        - "loss": Average loss over dataloader.
        - "mse": Mean squared error (averaged across dimensions).
        - "raw_mse": Per-dimension MSE (shape: [3]).
        - "rmse": Root mean squared error (averaged).
        - "raw_rmse": Per-dimension RMSE (shape: [3]).
        - "mae": Mean absolute error (averaged).
        - "raw_mae": Per-dimension MAE (shape: [3]).
        - "r2": R-squared score (averaged).
        - "raw_r2": Per-dimension R-squared (shape: [3]).
    """
    total_loss = 0
    count = 0
    targets = []
    preds = []

    model.eval()
    with torch.no_grad():
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            pred = model(x)
            loss = criterion(pred, y).item()
            total_loss += loss * x.shape[0]
            count += x.shape[0]
            pred, y = pred.detach().cpu(), y.detach().cpu()
            targets.append(y)
            preds.append(pred)

        targets = torch.cat(targets)
        preds = torch.cat(preds)

        error = preds - targets

        assert count == preds.shape[0]

        avg_loss = total_loss / count
        raw_mse = mean_squared_error(preds, targets, multioutput="raw_values")
        mse = mean_squared_error(
            preds, targets, multioutput="uniform_average"
        ).item()
        raw_rmse = torch.sqrt(raw_mse)
        rmse = raw_rmse.mean().item()
        raw_mae = torch.abs(error).mean(dim=0)
        mae = raw_mae.mean().item()
        raw_r2 = r2_score(preds, targets, multioutput="raw_values")
        r2 = r2_score(preds, targets, multioutput="uniform_average").item()

        sanity_raw_mse = torch.square(error).mean(dim=0)
        sanity_mse = sanity_raw_mse.mean().item()
        sanity_raw_rmse = torch.sqrt(sanity_raw_mse)
        sanity_rmse = sanity_raw_rmse.mean().item()

        assert np.isclose(sanity_mse, mse), (
            f"sanity_mse={sanity_mse:.4f}, mse={mse:.4f}"
        )
        assert np.isclose(sanity_rmse, rmse), (
            f"sanity_rmse={sanity_rmse:.4f}, rmse={rmse:.4f}"
        )

        return {
            "targets": targets,
            "preds": preds,
            "loss": avg_loss,
            "raw_mse": raw_mse,
            "mse": mse,
            "raw_rmse": raw_rmse,
            "rmse": rmse,
            "raw_mae": raw_mae,
            "mae": mae,
            "raw_r2": raw_r2,
            "r2": r2,
        }


def predict(
    session_dir: os.PathLike,
    config: Mapping[str, Any] | os.PathLike | str,
    visualize: bool = False,
) -> dict[str, Any]:
    """Run full prediction pipeline on test data.

    Loads a trained model and evaluates it on preprocessed test data,
    saving predictions and optionally visualizing results.

    Args:
        session_dir: Path to session directory containing preprocessed
            data and model weights.
        config: Path to YAML config file or config dict containing
            model parameters and paths.
        visualize: If True, creates 3D animation of predictions vs
            targets and saves as MP4.

    Returns:
        Dictionary from evaluate() containing predictions, targets, and
        all metrics (loss, mse, rmse, mae, r2, etc.).
    """
    # Load config
    if not isinstance(config, Mapping):
        with open(config, "r") as f:
            config = cast(Mapping[str, Any], yaml.safe_load(f))

    # Configure PyTorch
    seed_everything(50)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"Using device: {device}")
    configure_torch_dtype()

    # Load data
    data_path = os.path.join(session_dir, config["preprocess"]["filename"])
    df = pd.read_csv(data_path, index_col=False)

    logger.info(df.describe())

    # Initialize dataloaders
    loader = init_test_dataloader(df, **config["test_dataloader"])

    # Initialize criterion
    criterion = init_criterion(**config["criterion"]).to(device)

    # Initialize model, optimizer, and criterion
    model = init_model(**config["model"]).to(device)
    load_model_weights(model, config["weights_path"], device)

    results = evaluate(
        model=model,
        criterion=criterion,
        loader=loader,
        device=device,
    )

    # Print test results
    logger.info(
        f"Test results | "
        f"Loss: {results['loss']:.6f}, "
        f"MSE: {results['mse']:.6f}, "
        f"RMSE: {results['rmse']:.6f}, "
        f"MAE: {results['mae']:.6f}, "
        f"R2: {results['r2']:.6f}"
    )
    logger.info(
        f"Test results (raw values) | "
        f"MSE: {results['raw_mse']}, "
        f"RMSE: {results['raw_rmse']}, "
        f"MAE: {results['raw_mae']}, "
        f"R2: {results['raw_r2']}"
    )

    # Save the test targets and predictions
    targets = results["targets"].numpy()  # type: ignore
    preds = results["preds"].numpy()  # type: ignore
    df = pd.DataFrame(
        data=np.concatenate([targets, preds], axis=1),
        columns=[
            "target_x",
            "target_y",
            "target_z",
            "pred_x",
            "pred_y",
            "pred_z",
        ],  # type: ignore
    )
    df.to_csv(
        os.path.join(session_dir, config["predictions"]["filename"]),
        index=False,
    )

    if visualize:
        from tabletop_py.gaze.visualize import animate_3d_dots

        title = (
            f"Gaze Test Set Prediction\n"
            f"(RMSE={results['rmse']:.4f}, MAE={results['mae']:.4f})"
        )
        animate_3d_dots(
            {"Target": targets, "Prediction": preds},
            title=title,
            freq=config["eyelink_freq"],
            **config["visualize"]["animate_3d_dots"],
            save_path=os.path.join(session_dir, "predictions.mp4"),
        )

    return results


def main(args=None):
    import argparse

    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s - %(message)s"
    )

    parser = argparse.ArgumentParser(description="Train gaze estimation model")
    parser.add_argument(
        "-d",
        "--session-dir",
        type=str,
        default=os.path.join(os.environ["ROS_BAG_DIR"], "latest"),
        help="Path to bag directory",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default=os.path.join(
            os.environ["TABLETOP_DIR"], "config", "gaze_estimation.yaml"
        ),
        help="Path to model and training config file",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize the test targets and predictions",
    )
    args = parser.parse_args(args)

    # Attempt to load the calibration data from the session directory
    predict(**vars(args))


if __name__ == "__main__":
    main()
