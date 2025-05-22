import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from gaze_estimation import GazeEstimationModelMLP
from matplotlib.animation import FuncAnimation
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.model_selection import KFold, train_test_split
from sklearn.preprocessing import StandardScaler
from tqdm import tqdm


def load_data(eyelink_path, optitrack_path):
    """
    Loads and merges the eye tracker and optical marker data.

    Args:
        eyelink_path (str): Path to the eyelink data file.
        optitrack_path (str): Path to the optitrack data file.

    Returns:
        tuple: A tuple containing the merged data, input features (X), and target variables (y).
    """
    eyelink_data = pd.read_csv(eyelink_path)
    optitrack_data = pd.read_csv(optitrack_path)

    merged_data = pd.merge(eyelink_data, optitrack_data, on="time")
    merged_data = merged_data.dropna()

    X = merged_data[["left_x", "left_y", "right_x", "right_y"]].values
    y = merged_data[["X", "Y", "Z"]].values

    return merged_data, X, y


def init_model(device):
    model = GazeEstimationModelMLP().to(device)
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.MSELoss()
    return model, optimizer, criterion


def train_and_evaluate_model(
    X_train, y_train, X_test, y_test, device, y_scaler
):
    """
    Trains and evaluates the eye tracking model using cross-validation.

    Args:
        X_train (numpy.ndarray): Training input features.
        y_train (numpy.ndarray): Training target variables.
        X_test (numpy.ndarray): Test input features.
        y_test (numpy.ndarray): Test target variables.
        device (torch.device): Device to run the model on (CPU or GPU).
        y_scaler (sklearn.preprocessing.StandardScaler): Scaler for the target variables.

    Returns:
        tuple: A tuple containing the trained model, cross-validation scores, and test set metrics.
    """

    num_epochs = 5
    batch_size = 32
    patience = 30

    kf = KFold(n_splits=5, shuffle=False)

    mse_scores, rmse_scores, r2_scores = [], [], []
    best_model = None
    best_val_loss_fold = float("inf")

    for i, (train_index, val_index) in enumerate(kf.split(X_train)):
        print(f"Fold {i}")

        # Initialize model, optimizer, and criterion
        model = GazeEstimationModelMLP().to(device)
        optimizer = optim.Adam(model.parameters(), lr=0.001)
        criterion = nn.MSELoss()
        loader = torch.utils.data.DataLoader(
            torch.utils.data.TensorDataset(
                torch.tensor(X_train[train_index], dtype=torch.float32),
                torch.tensor(y_train[train_index], dtype=torch.float32),
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
            for X_batch, y_batch in tqdm(
                loader, desc=f"Training epoch {epoch} of {num_epochs}"
            ):
                X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                optimizer.zero_grad()
                outputs = model(X_batch)
                loss = criterion(outputs, y_batch)
                loss.backward()
                optimizer.step()

            model.eval()
            with torch.no_grad():
                X_val_fold = X_train[val_index]
                y_val_fold = y_train[val_index]
                y_val_pred = model(
                    torch.tensor(X_val_fold, dtype=torch.float32).to(device)
                )
                val_loss = criterion(
                    y_val_pred,
                    torch.tensor(y_val_fold, dtype=torch.float32).to(device),
                )
                print(f"Validation loss: {val_loss:.4f}")

            if val_loss < best_val_loss:
                best_val_loss = val_loss
                counter = 0
            else:
                counter += 1
                if counter >= patience:
                    print(
                        f"Fold {len(mse_scores) + 1}: Early stopping at epoch {epoch}"
                    )
                    break

        # Evaluate the model
        model.eval()
        with torch.no_grad():
            X_val_fold = X_train[val_index]
            y_val_fold = y_train[val_index]
            y_val_pred = model(
                torch.tensor(X_val_fold, dtype=torch.float32).to(device)
            )
            val_loss = criterion(
                y_val_pred,
                torch.tensor(y_val_fold, dtype=torch.float32).to(device),
            )
            y_val_pred = y_scaler.inverse_transform(y_val_pred.cpu().numpy())
            y_val_unscaled = y_scaler.inverse_transform(y_val_fold)
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

    return (
        best_model,
        (mse_scores, rmse_scores, r2_scores),
        test_set_metrics(X_test, y_test, model, device, y_scaler),
    )


def test_set_metrics(X_test, y_test, model, device, y_scaler):
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
    X_test_tensor = torch.tensor(X_test, dtype=torch.float32).to(device)
    y_test_tensor = torch.tensor(y_test, dtype=torch.float32).to(device)
    model.eval()
    with torch.no_grad():
        y_pred_scaled = model(X_test_tensor)
        y_pred = y_scaler.inverse_transform(y_pred_scaled.cpu().numpy())
        y_test_unscaled = y_scaler.inverse_transform(y_test)
        mse_test = mean_squared_error(y_test_unscaled, y_pred)
        rmse_test = np.sqrt(mse_test)
        r2_test = r2_score(y_test_unscaled, y_pred)
    return mse_test, rmse_test, r2_test


def plot_3d_animation(y_test, y_pred, mse_test, rmse_test, r2_test):
    """
    Plots a 3D animation of the actual and predicted values.

    Args:
        y_test (numpy.ndarray): Test target variables.
        y_pred (numpy.ndarray): Predicted target variables.
        mse_test (float): Test set MSE score.
        rmse_test (float): Test set RMSE score.
        r2_test (float): Test set R2 score.
    """
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    actual_scatter = ax.scatter([], [], [], c="blue", label="Actual")
    predicted_scatter = ax.scatter([], [], [], c="red", label="Predicted")

    ax.set_xlim(y_test[:, 0].min(), y_test[:, 0].max())
    ax.set_ylim(y_test[:, 1].min(), y_test[:, 1].max())
    ax.set_zlim(y_test[:, 2].min(), y_test[:, 2].max())
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    metrics_text = (
        f"MSE: {mse_test:.4f}\nRMSE: {rmse_test:.4f}\nR2: {r2_test:.4f}"
    )
    ax.text2D(
        0.98,
        0.02,
        metrics_text,
        transform=ax.transAxes,
        fontsize=10,
        ha="right",
        va="bottom",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    def update(frame):
        actual_x, actual_y, actual_z = y_test[frame]
        predicted_x, predicted_y, predicted_z = y_pred[frame]

        actual_scatter._offsets3d = ([actual_x], [actual_y], [actual_z])
        predicted_scatter._offsets3d = (
            [predicted_x],
            [predicted_y],
            [predicted_z],
        )

        return actual_scatter, predicted_scatter

    ani = FuncAnimation(
        fig, update, frames=len(y_test), interval=100, blit=True
    )

    plt.show()


def main():
    """
    Main function to run the gaze estimation model training and evaluation.
    """

    parser = argparse.ArgumentParser(description="Train gaze estimation model")
    parser.add_argument(
        "--eyelink-path",
        type=str,
        default="/root/ws/src/tabletop/testing/raw_tracking_data/5_21_24_t1_eyelink.csv",
        help="Path to eyelink data file",
    )
    parser.add_argument(
        "--optitrack-path",
        type=str,
        default="/root/ws/src/tabletop/testing/raw_tracking_data/5_21_24_t1_optitrack.csv",
        help="Path to optitrack data file",
    )
    args = parser.parse_args()

    eyelink_path = args.eyelink_path
    optitrack_path = args.optitrack_path

    print(f"Loading data from {eyelink_path} and {optitrack_path}")
    merged_data, X, y = load_data(eyelink_path, optitrack_path)

    print("Preprocessing data")
    X_scaler = StandardScaler()
    y_scaler = StandardScaler()
    X_scaled = X_scaler.fit_transform(X)
    y_scaled = y_scaler.fit_transform(y)

    X_train, X_test, y_train, y_test = train_test_split(
        X_scaled, y_scaled, test_size=0.2, random_state=42
    )

    # device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    device = torch.device("cpu")

    print("Training and evaluating model")
    model, cv_scores, test_scores = train_and_evaluate_model(
        X_train, y_train, X_test, y_test, device, y_scaler
    )

    mse_scores, rmse_scores, r2_scores = cv_scores
    mse_test, rmse_test, r2_test = test_scores

    # Retrieve predicted values for y using the final model
    model.eval()
    with torch.no_grad():
        X_test_tensor = torch.tensor(X_test, dtype=torch.float32).to(device)
        y_pred_scaled = model(X_test_tensor).cpu().numpy()
        y_pred = y_scaler.inverse_transform(y_pred_scaled)
        y_test_unscaled = y_scaler.inverse_transform(y_test)

    print("Cross-Validation Results (n_splits=5):")
    print(f"  MSE: {np.mean(mse_scores):.4f} ± {np.std(mse_scores):.4f}")
    print(f"  RMSE: {np.mean(rmse_scores):.4f} ± {np.std(rmse_scores):.4f}")
    print(f"  R2: {np.mean(r2_scores):.4f} ± {np.std(r2_scores):.4f}")
    print()
    print("Test Set Metrics:")
    print(f"  MSE: {mse_test:.4f}")
    print(f"  RMSE: {rmse_test:.4f}")
    print(f"  R2: {r2_test:.4f}")

    plot_3d_animation(y_test_unscaled, y_pred, mse_test, rmse_test, r2_test)


if __name__ == "__main__":
    main()
