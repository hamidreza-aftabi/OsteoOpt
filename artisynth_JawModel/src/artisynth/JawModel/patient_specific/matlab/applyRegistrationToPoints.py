import numpy as np
from pathlib import Path
import json

# Helper Functions
def load_deformation_weights(file_path):
    """
    Load deformation parameters from a JSON file.
    Expected keys:
        Y, W, beta
    """
    with open(file_path, "r") as f:
        params = json.load(f)

    params["Y"] = np.asarray(params["Y"], dtype=np.float64)
    params["W"] = np.asarray(params["W"], dtype=np.float64)
    params["beta"] = float(params["beta"])
    return params


def gaussian_kernel(X, Y, beta):
    diff = X[:, None, :] - Y[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    return np.exp(-dist2 / (2.0 * beta ** 2))


def reconstruct_control_shift(Y, W, beta):
    """
    Reconstruct TY - Y from saved CPD params:
        TY - Y = G(Y, Y) @ W
    """
    Y = np.asarray(Y, dtype=np.float64)
    W = np.asarray(W, dtype=np.float64)
    beta = float(beta)

    Gyy = gaussian_kernel(Y, Y, beta)
    return Gyy @ W


def apply_cpd_transform_to_points(points, Y, W, beta, chunk_size=10000):
    """
    Apply normalized difference-based CPD transfer to points:
        shift(points) = (G(points, Y) @ (TY - Y)) / row_sum
    """
    points = np.asarray(points, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    W = np.asarray(W, dtype=np.float64)
    beta = float(beta)

    control_shift = reconstruct_control_shift(Y, W, beta)

    out_pts = np.zeros_like(points)
    N = points.shape[0]

    for start in range(0, N, chunk_size):
        end = min(start + chunk_size, N)
        chunk = points[start:end]

        G = gaussian_kernel(chunk, Y, beta)

        row_sum = G.sum(axis=1, keepdims=True)
        row_sum[row_sum == 0] = 1e-12

        weighted_shift = (G @ control_shift) / row_sum
        out_pts[start:end] = chunk + weighted_shift

    return out_pts


def load_muscle_points(points_file):
    """
    Load muscle points (name, insertion point, origin point) from the points file.
    """
    muscle_points = {}
    with open(points_file, "r") as f:
        lines = f.readlines()

    for line in lines[1:]:
        parts = line.strip().split()
        muscle_name = parts[0]
        insertion_point = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=np.float64)
        origin_point = np.array([float(parts[4]), float(parts[5]), float(parts[6])], dtype=np.float64)
        muscle_points[muscle_name] = {
            "insertion": insertion_point,
            "origin": origin_point
        }

    return muscle_points


def transform_muscle_points(info_file, points_file, output_file, mandible_weights, maxilla_weights):
    """
    Transform insertion and origin points based on references in muscleInfo.txt.
    """
    muscle_points = load_muscle_points(points_file)

    with open(info_file, "r") as infile:
        lines = infile.readlines()

    transformed_data = ["MuscleName InsertionX InsertionY InsertionZ OriginX OriginY OriginZ"]

    for line in lines:
        if line.startswith("#") or len(line.strip()) == 0:
            continue

        parts = line.strip().split()
        muscle_name = parts[0]
        origin_reference = parts[1].lower()
        insertion_reference = parts[2].lower()

        if muscle_name not in muscle_points:
            raise ValueError(f"Muscle '{muscle_name}' not found in muscle_points file.")

        insertion_point = muscle_points[muscle_name]["insertion"]
        origin_point = muscle_points[muscle_name]["origin"]

        # Transform insertion point
        if insertion_reference in ["jaw", "jaw_resected"]:
            insertion_transformed = apply_cpd_transform_to_points(
                insertion_point[np.newaxis, :],
                mandible_weights["Y"],
                mandible_weights["W"],
                mandible_weights["beta"]
            )[0]
        elif insertion_reference == "skull":
            insertion_transformed = apply_cpd_transform_to_points(
                insertion_point[np.newaxis, :],
                maxilla_weights["Y"],
                maxilla_weights["W"],
                maxilla_weights["beta"]
            )[0]
        else:
            insertion_transformed = insertion_point

        # Transform origin point
        if origin_reference in ["jaw", "jaw_resected"]:
            origin_transformed = apply_cpd_transform_to_points(
                origin_point[np.newaxis, :],
                mandible_weights["Y"],
                mandible_weights["W"],
                mandible_weights["beta"]
            )[0]
        elif origin_reference == "skull":
            origin_transformed = apply_cpd_transform_to_points(
                origin_point[np.newaxis, :],
                maxilla_weights["Y"],
                maxilla_weights["W"],
                maxilla_weights["beta"]
            )[0]
        else:
            origin_transformed = origin_point

        transformed_data.append(
            f"{muscle_name} "
            f"{insertion_transformed[0]:.6f} {insertion_transformed[1]:.6f} {insertion_transformed[2]:.6f} "
            f"{origin_transformed[0]:.6f} {origin_transformed[1]:.6f} {origin_transformed[2]:.6f}"
        )

    with open(output_file, "w") as outfile:
        outfile.write("\n".join(transformed_data))


# Main Execution
if __name__ == "__main__":
    current_dir = Path().resolve()
    geometry_dir = current_dir.parent / "geometry"

    mandible_weights_file = current_dir / "deformation_weights_mandible.json"
    maxilla_weights_file = current_dir / "deformation_weights_maxilla.json"
    info_file = geometry_dir / "muscleInfo.txt"
    points_file = current_dir / "muscle_points.txt"
    output_file = current_dir / "transformed_muscle_points.txt"

    mandible_weights = load_deformation_weights(mandible_weights_file)
    maxilla_weights = load_deformation_weights(maxilla_weights_file)

    transform_muscle_points(
        info_file,
        points_file,
        output_file,
        mandible_weights,
        maxilla_weights
    )
    print(f"Transformed muscle points saved to: {output_file}")