import argparse
import os
from typing import Optional

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.model_selection import KFold, train_test_split
from sklearn.preprocessing import StandardScaler
from tqdm import tqdm

from tabletop_py.gaze.models import (
    GazeEstimationModelGeometric,
    GazeEstimationModelMLP,
)
from tabletop_py.gaze.preprocess import preprocess_data
from tabletop_py.gaze.visualization import animate_3d_dots


def load_data(session_dir: str) -> tuple[torch.Tensor, torch.Tensor]:
    """
    Loads the input features and target variables from the calibration data, or preprocesses the raw data if it doesn't exist.

    Args:
        session_dir (str): Path to the session directory.

    Returns:
        Tuple of input features (X), target variables (y)
    """
    path = os.path.join(session_dir, "calibration_data.csv")
    if not os.path.exists(path):
        data = preprocess_data(session_dir)
    else:
        data = pd.read_csv(path, index_col=False)

    X = data[["left_x", "left_y", "right_x", "right_y"]].values
    y = data[["marker_x", "marker_y", "marker_z"]].values

    X = torch.tensor(X, dtype=torch.float32)
    y = torch.tensor(y, dtype=torch.float32)

    return X, y


def init_model(
    model_type: str,
    lr: float = 0.001,
    device: Optional[torch.device] = None,
    **model_kwargs,
):
    if model_type == "mlp":
        model = GazeEstimationModelMLP(**model_kwargs).to(device)
    elif model_type == "geometric":
        model = GazeEstimationModelGeometric(**model_kwargs).to(device)
    else:
        raise ValueError(f"Invalid model type: {model_type}")

    optimizer = optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()
    return model, optimizer, criterion


def train(
    model_type: str,
    X_train: torch.Tensor,
    y_train: torch.Tensor,
    device: torch.device,
    y_scaler: StandardScaler,
    lr: float = 0.001,
    num_epochs: int = 20,
    batch_size: int = 128,
    patience_epochs: int = 5,
    **model_kwargs,
) -> tuple[
    GazeEstimationModelMLP | GazeEstimationModelGeometric,
    tuple[list[float], list[float], list[float]],
]:
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
    kf = KFold(n_splits=2, shuffle=False)

    mse_scores, rmse_scores, r2_scores = [], [], []
    best_model = None
    best_val_loss_fold = float("inf")

    for i, (train_index, val_index) in enumerate(kf.split(X_train)):
        print(f"Fold {i}")

        # Initialize model, optimizer, and criterion
        model, optimizer, criterion = init_model(
            model_type, lr, device, **model_kwargs
        )
        loader = torch.utils.data.DataLoader(
            torch.utils.data.TensorDataset(
                X_train[train_index], y_train[train_index]
            ),
            batch_size=batch_size,
            shuffle=True,
        )

        # Initialize best validation loss and counter
        best_val_loss = float("inf")
        counter = 0

        # Train the model
        model.train()
        for epoch in range(num_epochs):
            train_loss = 0
            for X_batch, y_batch in tqdm(
                loader, desc=f"Training epoch {epoch} of {num_epochs}"
            ):
                X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                optimizer.zero_grad()
                outputs = model(X_batch)
                loss = criterion(outputs, y_batch)
                loss.backward()
                optimizer.step()
                train_loss += loss.item()

            model.eval()
            with torch.no_grad():
                X_val_fold = X_train[val_index].to(device)
                y_val_fold = y_train[val_index].to(device)
                y_val_pred = model(X_val_fold)
                val_loss = criterion(y_val_pred, y_val_fold)
                print(f"Validation loss: {val_loss:.4f}")

            if val_loss < best_val_loss:
                best_val_loss = val_loss
                counter = 0
            else:
                counter += 1
                if counter >= patience_epochs:
                    print(
                        f"Fold {len(mse_scores) + 1}: Early stopping at epoch {epoch}"
                    )
                    break

        # Evaluate the model
        model.eval()
        with torch.no_grad():
            X_val_fold = X_train[val_index].to(device)
            y_val_fold = y_train[val_index].to(device)
            y_val_pred = model(X_val_fold)
            val_loss = criterion(y_val_pred, y_val_fold)
            y_val_pred = y_scaler.inverse_transform(y_val_pred.cpu().numpy())
            y_val_unscaled = y_scaler.inverse_transform(
                y_val_fold.cpu().numpy()
            )
            mse = mean_squared_error(y_val_unscaled, y_val_pred)
            rmse = np.sqrt(mse)
            r2 = r2_score(y_val_unscaled, y_val_pred)
            mse_scores.append(mse)
            rmse_scores.append(rmse)
            r2_scores.append(r2)
            print(
                f"Evaluation for fold {i} | Loss: {val_loss:.4f}, MSE: {mse:.4f}, RMSE: {rmse:.4f}, R2: {r2:.4f}"
            )
            if val_loss < best_val_loss_fold:
                best_val_loss_fold = val_loss
                best_model = model

    assert best_model is not None, "No best model found"

    return best_model, (mse_scores, rmse_scores, r2_scores)


def test(
    X_test: torch.Tensor,
    y_test: torch.Tensor,
    model: GazeEstimationModelMLP | GazeEstimationModelGeometric,
    y_scaler: StandardScaler,
    device: torch.device,
) -> tuple[float, float, float]:
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
    X_test = X_test.to(device)
    y_test = y_test.to(device)
    model.eval()
    with torch.no_grad():
        y_pred_scaled = model(X_test)
        y_pred = y_scaler.inverse_transform(y_pred_scaled.cpu().numpy())
        y_test_unscaled = y_scaler.inverse_transform(y_test.cpu().numpy())
        mse_test = mean_squared_error(y_test_unscaled, y_pred)
        rmse_test = np.sqrt(mse_test)
        r2_test = r2_score(y_test_unscaled, y_pred)

    return mse_test, rmse_test, r2_test


# def plot_3d_animation(y_test, y_pred, mse_test, rmse_test, r2_test):
#     """
#     Plots a 3D animation of the actual and predicted values.

#     Args:
#         y_test (numpy.ndarray): Test target variables.
#         y_pred (numpy.ndarray): Predicted target variables.
#         mse_test (float): Test set MSE score.
#         rmse_test (float): Test set RMSE score.
#         r2_test (float): Test set R2 score.
#     """
#     fig = plt.figure(figsize=(8, 6))
#     ax = fig.add_subplot(111, projection="3d")

#     actual_scatter = ax.scatter([], [], [], c="blue", label="Actual")
#     predicted_scatter = ax.scatter([], [], [], c="red", label="Predicted")

#     ax.set_xlim(y_test[:, 0].min(), y_test[:, 0].max())
#     ax.set_ylim(y_test[:, 1].min(), y_test[:, 1].max())
#     ax.set_zlim(y_test[:, 2].min(), y_test[:, 2].max())
#     ax.set_xlabel("X")
#     ax.set_ylabel("Y")
#     ax.set_zlabel("Z")
#     ax.legend()

#     metrics_text = (
#         f"MSE: {mse_test:.4f}\nRMSE: {rmse_test:.4f}\nR2: {r2_test:.4f}"
#     )
#     ax.text2D(
#         0.98,
#         0.02,
#         metrics_text,
#         transform=ax.transAxes,
#         fontsize=10,
#         ha="right",
#         va="bottom",
#         bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
#     )

#     def update(frame):
#         actual_x, actual_y, actual_z = y_test[frame]
#         predicted_x, predicted_y, predicted_z = y_pred[frame]

#         actual_scatter._offsets3d = ([actual_x], [actual_y], [actual_z])
#         predicted_scatter._offsets3d = (
#             [predicted_x],
#             [predicted_y],
#             [predicted_z],
#         )

#         return actual_scatter, predicted_scatter

#     ani = FuncAnimation(
#         fig, update, frames=len(y_test), interval=100, blit=True
#     )

#     plt.show()


def main(args=None):
    """
    Main function to run the gaze estimation model training and evaluation.
    """

    parser = argparse.ArgumentParser(description="Train gaze estimation model")
    parser.add_argument(
        "-d",
        "--session-dir",
        type=str,
        default=os.path.join(os.environ["ROS_BAG_DIR"], "latest"),
        help="Path to bag directory",
    )
    parser.add_argument(
        "--model",
        type=str,
        choices=["mlp", "geometric"],
        default="mlp",
        help="Model to use for gaze estimation",
    )
    args = parser.parse_args()

    X, y = load_data(args.session_dir)

    X_scaler = StandardScaler()
    y_scaler = StandardScaler()
    X_scaled = X_scaler.fit_transform(X)
    y_scaled = y_scaler.fit_transform(y)

    X_train, X_test, y_train, y_test = (
        torch.tensor(x, dtype=torch.float32)
        for x in train_test_split(
            X_scaled, y_scaled, test_size=0.2, shuffle=False, random_state=42
        )
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    print("Training and evaluating model")
    model, cv_scores = train(args.model, X_train, y_train, device, y_scaler)

    test_scores = test(X_test, y_test, model, y_scaler, device)

    mse_scores, rmse_scores, r2_scores = cv_scores
    mse_test, rmse_test, r2_test = test_scores

    print("Cross-Validation Results (n_splits=5):")
    print(f"  MSE: {np.mean(mse_scores):.4f} ± {np.std(mse_scores):.4f}")
    print(f"  RMSE: {np.mean(rmse_scores):.4f} ± {np.std(rmse_scores):.4f}")
    print(f"  R2: {np.mean(r2_scores):.4f} ± {np.std(r2_scores):.4f}")
    print()
    print("Test Set Metrics:")
    print(f"  MSE: {mse_test:.4f}")
    print(f"  RMSE: {rmse_test:.4f}")
    print(f"  R2: {r2_test:.4f}")

    # Retrieve predicted values for y using the final model
    model.eval()
    with torch.no_grad():
        y_pred_scaled = model(X_test.to(device)).cpu().numpy()
        y_pred = y_scaler.inverse_transform(y_pred_scaled)
        y_test_unscaled = y_scaler.inverse_transform(y_test)

    points = np.stack([y_test_unscaled, y_pred]).transpose(1, 0, 2)

    animate_3d_dots(points, freq=1000, save_path="test.mp4")


if __name__ == "__main__":
    main()
