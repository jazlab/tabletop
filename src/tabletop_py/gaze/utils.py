"""PyTorch utilities for gaze estimation model training.

This module provides utility functions and classes for training neural
network models for gaze estimation. It includes dataset handling,
dataloader initialization, and dynamic model/optimizer instantiation.

Functions:
    seed_everything: Set random seeds for reproducibility.
    configure_torch_dtype: Configure PyTorch default dtype and precision.
    init_dataloaders: Create train/val/test dataloaders with K-fold CV.
    init_model: Dynamically load model classes from gaze.models.
    init_optimizer: Dynamically load optimizers from torch.optim.
    init_criterion: Dynamically load loss functions from torch.nn.

Classes:
    GazeDataset: PyTorch Dataset for eye tracking data.

Example:
    seed_everything(42)
    configure_torch_dtype(torch.float32)
    train_val_gen, test_loader = init_dataloaders(df, test_size=0.2, ...)
    model = init_model("GazeEstimationModelMLP", input_size=4, ...)
"""

import importlib
import os
import random
from collections.abc import Generator
from typing import cast

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import KFold, train_test_split
from torch.utils.data import DataLoader, Dataset


def seed_everything(seed: int):
    """Set random seeds for reproducibility across all libraries.

    Sets seeds for PyTorch (CPU and CUDA), NumPy, and Python's random module.

    Args:
        seed: The random seed value.
    """
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    np.random.seed(seed)
    random.seed(seed)


def configure_torch_dtype(
    dtype: torch.dtype = torch.float32, matmul_precision: str = "high"
):
    """Configure PyTorch default dtype and matrix multiplication precision.

    Args:
        dtype: Default tensor dtype (default torch.float32).
        matmul_precision: Matrix multiplication precision level.
            Options: "highest", "high", "medium" (default "high").
    """
    torch.set_default_dtype(dtype)
    torch.set_float32_matmul_precision(matmul_precision)


class GazeDataset(Dataset):
    """PyTorch Dataset for gaze estimation training data.

    Loads eye tracking data (EYELINK_DATA_COLS) as inputs and motion capture
    marker positions (MARKER_DATA_COLS) as targets from a pandas DataFrame.

    Attributes:
        x: Input tensor of eye tracking data (shape: [N, num_input_features]).
        y: Target tensor of marker positions (shape: [N, 3]).
    """

    def __init__(self, df: pd.DataFrame):
        """Initialize the dataset from a DataFrame.

        Args:
            df: DataFrame containing both EYELINK_DATA_COLS and MARKER_DATA_COLS.
        """
        from tabletop_py.gaze.preprocess import (
            EYELINK_DATA_COLS,
            MARKER_DATA_COLS,
        )

        self.x = torch.tensor(
            df[EYELINK_DATA_COLS].to_numpy(), dtype=torch.get_default_dtype()
        )
        self.y = torch.tensor(
            df[MARKER_DATA_COLS].to_numpy(), dtype=torch.get_default_dtype()
        )

    def stats(self) -> dict[str, torch.Tensor]:
        """Compute dataset statistics for normalization.

        Returns:
            Dictionary with keys "x_mean", "x_std", "y_mean", "y_std"
            containing the mean and standard deviation for inputs and targets.
        """
        return {
            "x_mean": self.x.mean(dim=0),
            "x_std": self.x.std(dim=0),
            "y_mean": self.y.mean(dim=0),
            "y_std": self.y.std(dim=0),
        }

    def __len__(self):
        """Return the number of samples in the dataset."""
        return self.x.shape[0]

    def __getitem__(self, idx):
        """Get a single sample by index.

        Args:
            idx: Sample index.

        Returns:
            Tuple of (input_features, target_position).
        """
        return self.x[idx], self.y[idx]


def init_dataloaders(
    df: pd.DataFrame,
    test_size: float,
    val_folds: int,
    shuffle_test_split: bool,
    shuffle_val_split: bool,
    train_batch_size: int,
    val_batch_size: int,
    test_batch_size: int,
    num_workers: int,
) -> tuple[Generator[tuple[DataLoader, DataLoader], None, None], DataLoader]:
    """
    Initializes the dataloaders for the training, validation, and test sets.

    Args:
        df: The dataframe to split.
        test_size: The size of the test set.
        val_folds: The number of folds for the validation set.
        shuffle_test_split: Whether to shuffle the test set.
        shuffle_val_split: Whether to shuffle the validation set.
        train_batch_size: The batch size for the training set.
        val_batch_size: The batch size for the validation set.
        test_batch_size: The batch size for the test set.

    Returns:
        A tuple containing a generator of train and validation dataloaders and the test dataloader.
    """

    train_val_df, test_df = train_test_split(
        df, test_size=test_size, shuffle=shuffle_test_split
    )
    train_val_df = cast(pd.DataFrame, train_val_df)
    test_df = cast(pd.DataFrame, test_df)

    def train_val_generator() -> Generator[
        tuple[DataLoader, DataLoader], None, None
    ]:
        kf = KFold(n_splits=val_folds, shuffle=shuffle_val_split)
        for train_idx, val_idx in kf.split(train_val_df):
            train_dataset = GazeDataset(train_val_df.iloc[train_idx])
            val_dataset = GazeDataset(train_val_df.iloc[val_idx])
            train_loader = DataLoader(
                train_dataset,
                batch_size=train_batch_size,
                shuffle=True,
                num_workers=num_workers,
            )
            val_loader = DataLoader(
                val_dataset,
                batch_size=val_batch_size,
                shuffle=False,
                num_workers=num_workers,
            )
            yield train_loader, val_loader

    test_dataset = GazeDataset(test_df)
    test_loader = DataLoader(
        test_dataset,
        batch_size=test_batch_size,
        shuffle=False,
        num_workers=num_workers,
    )

    return train_val_generator(), test_loader


def init_test_dataloader(
    df: pd.DataFrame,
    batch_size: int,
    num_workers: int,
) -> DataLoader:
    """
    TODO
    """
    dataset = GazeDataset(df)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
    )
    return loader


def init_model(name: str, **kwargs) -> nn.Module:
    """Dynamically instantiate a model class from tabletop_py.gaze.models.

    Args:
        name: Name of the model class (e.g., "GazeEstimationModelMLP").
        **kwargs: Arguments passed to the model constructor.

    Returns:
        Instantiated model.
    """
    model_class: type[nn.Module] = getattr(
        importlib.import_module("tabletop_py.gaze.models"), name
    )
    model = model_class(**kwargs)
    return model


def load_model_weights(
    model: nn.Module, weights_path: str, device: str | torch.device
):
    weights_path = os.path.expanduser(os.path.expandvars(weights_path))
    state_dict = torch.load(weights_path, map_location=device)
    model.load_state_dict(state_dict)


def init_optimizer(model: nn.Module, name: str, **kwargs) -> optim.Optimizer:
    """Dynamically instantiate an optimizer from torch.optim.

    Args:
        model: Model whose parameters will be optimized.
        name: Name of the optimizer class (e.g., "Adam", "SGD").
        **kwargs: Arguments passed to the optimizer constructor.

    Returns:
        Instantiated optimizer.
    """
    optimizer_class: type[optim.Optimizer] = getattr(
        importlib.import_module("torch.optim"),
        name,  # type: ignore
    )
    optimizer = optimizer_class(model.parameters(), **kwargs)
    return optimizer


def init_criterion(name: str, **kwargs) -> nn.Module:
    """Dynamically instantiate a loss criterion from torch.nn.

    Args:
        name: Name of the loss class (e.g., "MSELoss", "L1Loss").
        **kwargs: Arguments passed to the loss constructor.

    Returns:
        Instantiated loss function.
    """
    criterion_class: type[nn.Module] = getattr(
        importlib.import_module("torch.nn"), name
    )
    criterion = criterion_class(**kwargs)
    return criterion
