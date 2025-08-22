import importlib
from collections.abc import Generator
from typing import cast

import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import KFold, train_test_split
from sklearn.preprocessing import StandardScaler
from torch.utils.data import DataLoader, Dataset


class GazeDataset(Dataset):
    def __init__(self, df: pd.DataFrame):
        x = df[["left_x", "left_y", "right_x", "right_y"]].values
        y = df[["marker_x", "marker_y", "marker_z"]].values
        self.x_scaler = StandardScaler()
        self.y_scaler = StandardScaler()
        self.x = torch.from_numpy(self.x_scaler.fit_transform(x)).to(
            torch.float32
        )
        self.y = torch.from_numpy(self.y_scaler.fit_transform(y)).to(
            torch.float32
        )

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        return self.x[idx], self.y[idx]

    def unscale_x(self, x: torch.Tensor) -> torch.Tensor:
        return torch.from_numpy(
            self.x_scaler.inverse_transform(x.detach().cpu())
        )

    def unscale_y(self, y: torch.Tensor) -> torch.Tensor:
        return torch.from_numpy(
            self.y_scaler.inverse_transform(y.detach().cpu())
        )


def init_dataloaders(
    df: pd.DataFrame,
    test_size: float,
    val_folds: int,
    shuffle_test_split: bool,
    shuffle_val_split: bool,
    train_batch_size: int,
    val_batch_size: int,
    test_batch_size: int,
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
                train_dataset, batch_size=train_batch_size, shuffle=True
            )
            val_loader = DataLoader(
                val_dataset, batch_size=val_batch_size, shuffle=False
            )
            yield train_loader, val_loader

    test_dataset = GazeDataset(test_df)  # type: ignore
    test_loader = DataLoader(
        test_dataset, batch_size=test_batch_size, shuffle=False
    )

    return train_val_generator(), test_loader


def init_model(class_name: str, **kwargs) -> nn.Module:
    model_class: type[nn.Module] = getattr(
        importlib.import_module("tabletop_py.gaze.models"), class_name
    )
    model = model_class(**kwargs)
    return model


def init_optimizer(
    model: nn.Module, class_name: str, **kwargs
) -> optim.Optimizer:
    optimizer_class: type[optim.Optimizer] = getattr(
        importlib.import_module("torch.optim"),
        class_name,  # type: ignore
    )
    optimizer = optimizer_class(model.parameters(), **kwargs)
    return optimizer


def init_criterion(class_name: str, **kwargs) -> nn.Module:
    criterion_class: type[nn.Module] = getattr(
        importlib.import_module("torch.nn"), class_name
    )
    criterion = criterion_class(**kwargs)
    return criterion
