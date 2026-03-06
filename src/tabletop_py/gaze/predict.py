"""Training and evaluation pipeline for gaze estimation models.

This module provides the training loop, evaluation metrics, and full
training pipeline for gaze estimation models. Supports K-fold cross-
validation with early stopping and model checkpointing.

Functions:
    evaluate: Compute test metrics (MSE, RMSE, R2) on a dataloader.
    train: Train a model with early stopping on validation loss.
    train_and_evaluate: Full pipeline with cross-validation and test eval.
    main: CLI entry point for training.

The training pipeline:
1. Load and preprocess data from session directory
2. Initialize K-fold cross-validation dataloaders
3. Train models for each fold with early stopping
4. Select best model based on validation loss
5. Evaluate on held-out test set
6. Save model weights and predictions

Example:
    python -m tabletop_py.gaze.train -d /path/to/session --visualize
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
    """
    Calculates the test set metrics (MSE, RMSE, R2) for the trained model.

    Args:
        X_test (numpy.ndarray): Test input features.
        y_test (numpy.ndarray): Test target variables.
        model (torch.nn.Module): Trained model.
        device (torch.device): Device to run the model on (CPU or GPU).
        y_scaler (sklearn.preprocessing.StandardScaler): Scaler for the target variables.

    Returns:
        tuple: A tuple containing the test set MSE, RMSE, and R2 scores.
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
    """
    Trains and evaluates the gaze estimation model.
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
    load_model_weights(model, config["weights_path"])

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
