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
import torch.optim as optim
import torcheval.metrics as metrics
import yaml

from tabletop_py.gaze.utils import (
    configure_torch_dtype,
    init_criterion,
    init_dataloaders,
    init_model,
    init_optimizer,
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
    eval_loss = 0
    count = 0
    mse = metrics.MeanSquaredError()
    r2 = metrics.R2Score()
    targets = []
    preds = []

    model.eval()
    with torch.no_grad():
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            pred = model(x)
            loss = criterion(pred, y).item()
            eval_loss += loss * x.shape[0]
            count += x.shape[0]
            pred, y = pred.detach().cpu(), y.detach().cpu()
            mse.update(pred, y)
            r2.update(pred, y)
            targets.append(y)
            preds.append(pred)

    eval_loss /= count
    mse = mse.compute()
    rmse = torch.sqrt(mse)
    r2 = r2.compute()

    targets = torch.cat(targets)
    preds = torch.cat(preds)

    return {
        "targets": targets,
        "preds": preds,
        "loss": eval_loss,
        "mse": mse.item(),
        "rmse": rmse.item(),
        "r2": r2.item(),
    }


def train(
    model: nn.Module,
    optimizer: optim.Optimizer,
    criterion: nn.Module,
    train_loader: torch.utils.data.DataLoader,
    val_loader: torch.utils.data.DataLoader,
    *,
    num_epochs: int,
    patience_epochs: int,
    device: torch.device,
) -> dict[str, float | torch.Tensor]:
    """
    Trains the eye tracking model using cross-validation.

    Args:
        X_train: Training input features.
        y_train: Training target variables.
        device: Device to run the model on (CPU or GPU).
        y_scaler: Scaler for the target variables.
        model_type: Type of model to use for gaze estimation.

    Returns:
        tuple: A tuple containing the trained model and cross-validation scores.
    """

    best_val_loss = float("inf")
    patience_counter = 0

    # Train the model
    for epoch in range(num_epochs):
        train_loss = 0
        count = 0
        model.train()
        for x, y in train_loader:
            x, y = x.to(device), y.to(device)
            optimizer.zero_grad()
            pred = model(x)
            loss = criterion(pred, y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * x.shape[0]
            count += x.shape[0]

        train_loss /= count

        val_results = evaluate(
            model=model,
            criterion=criterion,
            loader=val_loader,
            device=device,
        )
        val_loss = val_results["loss"]

        logger.info(
            f"Epoch {epoch + 1} of {num_epochs} | "
            f"Train loss: {train_loss:.6f} | "
            f"Validation loss: {val_loss:.6f} | "
            f"Validation RMSE: {val_results['rmse']:.6f} | "
            f"Patience counter: {patience_counter}/{patience_epochs}"
        )

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
        else:
            patience_counter += 1
            if patience_counter >= patience_epochs:
                logger.info(
                    f"Early stopping at epoch {epoch + 1} of {num_epochs}"
                )
                break

    return val_results


def train_and_evaluate(
    session_dir: os.PathLike,
    config: Mapping[str, Any] | os.PathLike | str,
    visualize: bool = False,
) -> dict[str, Any]:
    """
    Trains and evaluates the gaze estimation model.
    """
    # Configure PyTorch
    seed_everything(50)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"Using device: {device}")
    configure_torch_dtype()

    # Load config
    if not isinstance(config, Mapping):
        with open(config, "r") as f:
            config = cast(Mapping[str, Any], yaml.safe_load(f))

    # Load data
    path = os.path.join(session_dir, config["preprocess"]["filename"])
    df = pd.read_csv(path, index_col=False)

    logger.info(df.describe())

    # Initialize dataloaders
    train_val_loader_generator, test_loader = init_dataloaders(
        df, **config["dataloaders"]
    )

    # Initialize best model and best validation loss
    best_fold: int | None = None
    best_model: nn.Module | None = None
    best_val_results: dict[str, float | torch.Tensor] | None = None
    best_val_loss = float("inf")
    max_folds = config["train"].pop("max_folds")

    # Train and evaluate the model
    for i, (train_loader, val_loader) in enumerate(train_val_loader_generator):
        if i >= max_folds:
            break

        logger.info(
            f"Training and evaluating fold {i}/{config['dataloaders']['val_folds']}"
        )

        train_data_stats = train_loader.dataset.stats()  # type: ignore

        # Initialize model, optimizer, and criterion
        model = init_model(
            input_mean=train_data_stats["x_mean"],
            input_std=train_data_stats["x_std"],
            output_mean=train_data_stats["y_mean"],
            output_std=train_data_stats["y_std"],
            **config["model"],
        ).to(device)
        model.compile(**config["compile"])

        optimizer = init_optimizer(model=model, **config["optimizer"])
        criterion = init_criterion(**config["criterion"]).to(device)

        # Train the model
        val_results = train(
            model=model,
            optimizer=optimizer,
            criterion=criterion,
            train_loader=train_loader,
            val_loader=val_loader,
            **config["train"],
            device=device,
        )

        logger.info(
            f"Validation results for fold {i} | "
            f"Loss: {val_results['loss']:.6f}, "
            f"MSE: {val_results['mse']:.6f}, "
            f"RMSE: {val_results['rmse']:.6f}, "
            f"R2: {val_results['r2']:.6f}"
        )
        if val_results["loss"] < best_val_loss:
            best_fold = i
            best_model = model
            best_val_results = val_results
            best_val_loss = val_results["loss"]

    assert (
        best_model is not None
        and best_val_results is not None
        and best_fold is not None
    )
    test_results = evaluate(
        model=best_model,
        criterion=criterion,
        loader=test_loader,
        device=device,
    )

    # Print test results
    logger.info(
        f"Test results for best model from fold {best_fold} | "
        f"Loss: {test_results['loss']:.6f}, "
        f"MSE: {test_results['mse']:.6f}, "
        f"RMSE: {test_results['rmse']:.6f}, "
        f"R2: {test_results['r2']:.6f}"
    )

    # Save the best model
    path = os.path.expandvars(config["weights_path"])
    path = os.path.expanduser(path)
    if not os.path.isabs(path):
        path = os.path.join(os.environ["TABLETOP_DIR"], path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    torch.save(best_model.state_dict(), path)
    logger.info(f"Saved best model to {path}")

    # Save the test targets and predictions
    targets = test_results["targets"].numpy()  # type: ignore
    preds = test_results["preds"].numpy()  # type: ignore
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

        animate_3d_dots(
            {"Target": targets, "Prediction": preds},
            freq=config["eyelink_freq"],
            **config["visualize"]["animate_3d_dots"],
            save_path=os.path.join(session_dir, "predictions.mp4"),
        )

    return {
        "best_model": best_model,
        "best_val_results": best_val_results,
        "test_results": test_results,
    }


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
    parser.add_argument(
        "--predict",
        action="store_true",
        help="Predict the gaze from the model",
    )
    args = parser.parse_args(args)

    # Attempt to load the calibration data from the session directory
    train_and_evaluate(**vars(args))


if __name__ == "__main__":
    main()
