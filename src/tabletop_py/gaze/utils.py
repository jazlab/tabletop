import importlib
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
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    np.random.seed(seed)
    random.seed(seed)


def configure_torch_dtype(
    dtype: torch.dtype = torch.float32, matmul_precision: str = "high"
):
    torch.set_default_dtype(dtype)
    torch.set_float32_matmul_precision(matmul_precision)


class GazeDataset(Dataset):
    def __init__(self, df: pd.DataFrame):
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
        return {
            "x_mean": self.x.mean(dim=0),
            "x_std": self.x.std(dim=0),
            "y_mean": self.y.mean(dim=0),
            "y_std": self.y.std(dim=0),
        }

    def __len__(self):
        return self.x.shape[0]

    def __getitem__(self, idx):
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

    def train_val_generator() -> (
        Generator[tuple[DataLoader, DataLoader], None, None]
    ):
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

    test_dataset = GazeDataset(test_df)  # type: ignore
    test_loader = DataLoader(
        test_dataset, batch_size=test_batch_size, shuffle=False
    )

    return train_val_generator(), test_loader


def init_model(name: str, **kwargs) -> nn.Module:
    model_class: type[nn.Module] = getattr(
        importlib.import_module("tabletop_py.gaze.models"), name
    )
    model = model_class(**kwargs)
    return model


def init_optimizer(model: nn.Module, name: str, **kwargs) -> optim.Optimizer:
    optimizer_class: type[optim.Optimizer] = getattr(
        importlib.import_module("torch.optim"),
        name,  # type: ignore
    )
    optimizer = optimizer_class(model.parameters(), **kwargs)
    return optimizer


def init_criterion(name: str, **kwargs) -> nn.Module:
    criterion_class: type[nn.Module] = getattr(
        importlib.import_module("torch.nn"), name
    )
    criterion = criterion_class(**kwargs)
    return criterion
