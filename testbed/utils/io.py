from __future__ import annotations

import numpy as np
import scipy.io as spio


def load_data_matlab(
    filename: str,
    split_data: int = 0,
    shift_x: float = -200,
    scale_x: float = 1.3,
    shift_y: float = -85,
    scale_y: float = 30,
):
    """Load trajectory data from MATLAB and return a position callback."""
    if not isinstance(filename, str):
        raise TypeError("filename must be a string")
    if not isinstance(split_data, int):
        raise TypeError("split_data must be an integer")
    if split_data < 0:
        raise ValueError("split_data must be >= 0")

    mat = spio.loadmat(filename, squeeze_me=True)
    hist_pos = np.asarray(mat["hist_pos"], dtype=float)
    zhist = np.asarray(mat["zhist"], dtype=float)

    if hist_pos.ndim != 2 or zhist.ndim != 2:
        raise ValueError("hist_pos and zhist must be 2D matrices")
    if hist_pos.shape != zhist.shape:
        raise ValueError("hist_pos and zhist must have the same shape")

    n, horizon = hist_pos.shape

    if split_data > 0 and horizon > 1:
        x_pos = np.zeros((n, (horizon - 1) * split_data), dtype=float)
        y_pos = np.zeros((n, (horizon - 1) * split_data), dtype=float)
        for j in range(horizon - 1):
            dtx = (hist_pos[:, j + 1] - hist_pos[:, j]) / split_data
            dty = (zhist[:, j + 1] - zhist[:, j]) / split_data
            for k in range(split_data):
                x_pos[:, j * split_data + k] = hist_pos[:, j] + k * dtx
                y_pos[:, j * split_data + k] = zhist[:, j] + k * dty
    else:
        x_pos = hist_pos
        y_pos = zhist

    def position(i: int) -> np.ndarray:
        if i < 0 or i >= x_pos.shape[1]:
            raise IndexError("trajectory index out of range")
        return np.array(
            [
                x_pos[:, i] * scale_x + shift_x,
                y_pos[:, i] * scale_y + shift_y,
                np.zeros((n,), dtype=float),
            ]
        )

    return position
