import numpy as np
import json
from pathlib import Path


def load_deformation_weights(file_path):
    """
    Load deformation parameters from a JSON file.
    Expected keys:
        Y: control points
        W: CPD weights
        beta: Gaussian kernel width
    """
    with open(file_path, "r") as f:
        params = json.load(f)

    params["Y"] = np.asarray(params["Y"], dtype=np.float64)
    params["W"] = np.asarray(params["W"], dtype=np.float64)
    params["beta"] = float(params["beta"])

    return params


def gaussian_kernel(X, Y, beta):
    """
    Compute Gaussian kernel matrix:
        G_ij = exp(-||X_i - Y_j||^2 / (2 * beta^2))
    """
    diff = X[:, None, :] - Y[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    return np.exp(-dist2 / (2.0 * beta ** 2))


def reconstruct_transformed_control_points(Y, W, beta):
    """
    Reconstruct transformed control points for CPD:
        TY = Y + G(Y, Y) @ W
    """
    G = gaussian_kernel(Y, Y, beta)
    TY = Y + G @ W
    return TY


def extract_translation_scaling(Y, TY):
    """
    Approximate a global translation and isotropic scaling between Y and TY.

    Translation:
        Mean displacement between TY and Y.

    Scaling:
        Best-fit isotropic scale after removing centroids.
    """
    Y = np.asarray(Y, dtype=np.float64)
    TY = np.asarray(TY, dtype=np.float64)

    if Y.shape != TY.shape:
        raise ValueError(f"Y and TY must have same shape. Got {Y.shape} and {TY.shape}")

    # Approximate translation = mean displacement
    translation = np.mean(TY - Y, axis=0)

    # Remove centroids for scale estimation
    Y_center = Y - np.mean(Y, axis=0)
    TY_center = TY - np.mean(TY, axis=0)

    denom = np.sum(Y_center ** 2)
    if denom < 1e-12:
        scaling = 1.0
    else:
        scaling = np.sqrt(np.sum(TY_center ** 2) / denom)

    return translation, float(scaling)


def save_transformation(translation, scaling, output_file):
    """
    Save the approximate translation and scaling to a JSON file.
    """
    transformation = {
        "translation": np.asarray(translation, dtype=np.float64).tolist(),
        "scaling": float(scaling)
    }
    with open(output_file, "w") as f:
        json.dump(transformation, f, indent=4)

    print(f"Transformation saved to {output_file}")


# Main Execution
if __name__ == "__main__":
    # Paths
    current_dir = Path().resolve()
    weights_file = current_dir / "deformation_weights_mandible.json"
    output_file = current_dir / "translation_scaling.json"

    # Load deformation parameters
    weights = load_deformation_weights(weights_file)

    # Reconstruct TY from Y, W, beta
    TY = reconstruct_transformed_control_points(
        weights["Y"],
        weights["W"],
        weights["beta"]
    )

    # Extract approximate translation and scaling
    translation, scaling = extract_translation_scaling(weights["Y"], TY)

    # Save result
    save_transformation(translation, scaling, output_file)