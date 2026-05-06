import numpy as np
import trimesh
from scipy.spatial import cKDTree
from pathlib import Path
import json

# ============================================================
# Flags
# ============================================================
ENABLE_RIGID = False
ENABLE_AXIS_SCALING = False
ENABLE_DUAL_DEFORMABLE = True

# ============================================================
# Paths
# ============================================================
current_dir = Path().resolve()
geometry_dir = current_dir.parent / "geometry"

disc_path = str(geometry_dir / "disc_right.obj")
condyle_params_file = str(current_dir / "registration_pipeline_condyle_right.json")
fossa_params_file = str(current_dir / "registration_pipeline_fossa_right.json")
out_path = str(geometry_dir / "disc_dual_deformed_right.obj")

# ============================================================
# Weighting params
# ============================================================
K_NEIGHBORS = 20
WEIGHT_POWER = .5
EPS = 1e-8
CHUNK_SIZE = 5000

# Manual influence adjustment
# > 1.0 = stronger influence
# < 1.0 = weaker influence
CONDYLE_WEIGHT_FACTOR = 1
FOSSA_WEIGHT_FACTOR = 1


# ============================================================
# Helper Functions
# ============================================================
def load_mesh_as_nx3(path):
    mesh = trimesh.load(path, process=False)
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError(f"Mesh '{path}' must have shape (N,3). Got {verts.shape}")
    return mesh, verts


def remove_degenerate_faces(vertices, faces):
    valid_faces = [f for f in faces if len(set(f)) == len(f)]
    valid_faces = np.asarray(valid_faces, dtype=np.int32)

    mesh = trimesh.Trimesh(vertices=vertices, faces=valid_faces, process=False)
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    return np.asarray(mesh.vertices), np.asarray(mesh.faces)


def save_mesh(path, vertices, faces):
    if faces is None or len(faces) == 0:
        with open(path, "w") as f:
            for v in vertices:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
    else:
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        mesh.export(path)


def apply_rigid_transform(points, rigid_params):
    s = float(rigid_params["s"])
    R = np.asarray(rigid_params["R"], dtype=np.float64)
    t = np.asarray(rigid_params["t"], dtype=np.float64)
    return s * (points @ R) + t


def apply_linear_transform_full(points, B, t):
    B = np.asarray(B, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)
    return points @ B + t


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


def compute_normalized_transfer_shift(points, deformable_params, chunk_size=5000):
    """
    Compute normalized difference-based transfer shift:
        shift(points) = (G(points, Y) @ (TY - Y)) / row_sum

    Supports:
      - single-stage dict
      - multi-stage list
    """
    points = np.asarray(points, dtype=np.float64)
    current = points.copy()

    if deformable_params is None:
        return np.zeros_like(points)

    if isinstance(deformable_params, dict):
        stages = [deformable_params]
    elif isinstance(deformable_params, list):
        stages = deformable_params
    else:
        raise TypeError(f"Unsupported deformable_params type: {type(deformable_params)}")

    for i, stage in enumerate(stages, start=1):
        if stage is None or not stage.get("enabled", False):
            continue

        if "W" not in stage:
            raise ValueError(
                f"Deformable stage {i} is missing 'W'. "
                "Please regenerate the JSON using the revised registration code."
            )

        Y = np.asarray(stage["Y"], dtype=np.float64)
        W = np.asarray(stage["W"], dtype=np.float64)
        beta = float(stage["beta"])

        control_shift = reconstruct_control_shift(Y, W, beta)

        out_shift = np.zeros_like(current)
        N = current.shape[0]

        for start in range(0, N, chunk_size):
            end = min(start + chunk_size, N)
            chunk = current[start:end]

            G = gaussian_kernel(chunk, Y, beta)
            row_sum = G.sum(axis=1, keepdims=True)
            row_sum[row_sum == 0] = 1e-12

            out_shift[start:end] = (G @ control_shift) / row_sum

        current = current + out_shift

    return current - points


def get_reference_points_for_weighting(deformable_params):
    """
    Pick reference points for regional weighting.

    For multi-stage deformable pipelines, use the last stage's control points.
    """
    if deformable_params is None:
        raise ValueError("deformable_params is None")

    if isinstance(deformable_params, dict):
        return np.asarray(deformable_params["Y"], dtype=np.float64)

    if isinstance(deformable_params, list):
        enabled_stages = [stage for stage in deformable_params if stage is not None and stage.get("enabled", False)]
        if len(enabled_stages) == 0:
            raise ValueError("No enabled deformable stages found")
        return np.asarray(enabled_stages[-1]["Y"], dtype=np.float64)

    raise TypeError(f"Unsupported deformable_params type: {type(deformable_params)}")


def query_mean_k_distance(tree, points, k=3):
    dists, _ = tree.query(points, k=k)
    if k == 1:
        dists = dists.reshape(-1, 1)
    return np.mean(dists, axis=1)


def compute_region_weights(points,
                           condyle_reference_points,
                           fossa_reference_points,
                           k_neighbors=3,
                           power=2.0,
                           eps=1e-8,
                           condyle_factor=1.0,
                           fossa_factor=1.0):
    condyle_reference_points = np.asarray(condyle_reference_points, dtype=np.float64)
    fossa_reference_points = np.asarray(fossa_reference_points, dtype=np.float64)

    if condyle_reference_points.ndim != 2 or condyle_reference_points.shape[1] != 3:
        raise ValueError("condyle_reference_points must have shape (N,3)")
    if fossa_reference_points.ndim != 2 or fossa_reference_points.shape[1] != 3:
        raise ValueError("fossa_reference_points must have shape (N,3)")
    if len(condyle_reference_points) == 0:
        raise ValueError("condyle_reference_points is empty")
    if len(fossa_reference_points) == 0:
        raise ValueError("fossa_reference_points is empty")
    if condyle_factor < 0 or fossa_factor < 0:
        raise ValueError("condyle_factor and fossa_factor must be >= 0")

    kc = min(k_neighbors, len(condyle_reference_points))
    kf = min(k_neighbors, len(fossa_reference_points))

    condyle_tree = cKDTree(condyle_reference_points)
    fossa_tree = cKDTree(fossa_reference_points)

    d_condyle = query_mean_k_distance(condyle_tree, points, k=kc)
    d_fossa = query_mean_k_distance(fossa_tree, points, k=kf)

    inv_condyle = 1.0 / ((d_condyle + eps) ** power)
    inv_fossa = 1.0 / ((d_fossa + eps) ** power)

    inv_condyle = condyle_factor * inv_condyle
    inv_fossa = fossa_factor * inv_fossa

    denom = inv_condyle + inv_fossa
    denom[denom == 0] = 1e-12

    w_condyle = (inv_condyle / denom).reshape(-1, 1)
    w_fossa = (inv_fossa / denom).reshape(-1, 1)

    return w_condyle, w_fossa


# ============================================================
# Load disc mesh
# ============================================================
dmesh, dverts = load_mesh_as_nx3(disc_path)
dfaces = np.asarray(dmesh.faces) if dmesh.faces is not None and len(dmesh.faces) > 0 else None

# ============================================================
# Load saved parameter files
# ============================================================
with open(condyle_params_file, "r") as f:
    condyle_params = json.load(f)

with open(fossa_params_file, "r") as f:
    fossa_params = json.load(f)

# ============================================================
# Start from original disc
# ============================================================
full_transformed = dverts.copy()

# ============================================================
# Optional rigid from condyle file
# ============================================================
rigid_params = condyle_params.get("rigid", None)
if ENABLE_RIGID and rigid_params is not None and rigid_params.get("enabled", False):
    full_transformed = apply_rigid_transform(full_transformed, rigid_params)
    print("Applied rigid transform.")

# ============================================================
# Optional axis scaling from condyle file
# ============================================================
axis_scaling_params = condyle_params.get("axis_scaling", None)
if ENABLE_AXIS_SCALING and axis_scaling_params is not None and axis_scaling_params.get("enabled", False):
    B = np.asarray(axis_scaling_params["B"], dtype=np.float64)
    t = np.asarray(axis_scaling_params["t"], dtype=np.float64)
    full_transformed = apply_linear_transform_full(full_transformed, B, t)
    print("Applied axis scaling transform.")

# ============================================================
# Dual deformable
# ============================================================
if ENABLE_DUAL_DEFORMABLE:
    deformable_condyle = condyle_params.get("deformable", None)
    deformable_fossa = fossa_params.get("deformable", None)

    if deformable_condyle is None:
        raise ValueError("Condyle JSON does not contain deformable parameters.")
    if deformable_fossa is None:
        raise ValueError("Fossa JSON does not contain deformable parameters.")

    # Reference points used only for spatial weighting
    condyle_reference_points = get_reference_points_for_weighting(deformable_condyle)
    fossa_reference_points = get_reference_points_for_weighting(deformable_fossa)

    # Compute normalized transfer shifts from both deformable pipelines
    shift_condyle = compute_normalized_transfer_shift(
        full_transformed,
        deformable_condyle,
        chunk_size=CHUNK_SIZE
    )

    shift_fossa = compute_normalized_transfer_shift(
        full_transformed,
        deformable_fossa,
        chunk_size=CHUNK_SIZE
    )

    # Spatial blending weights
    w_condyle, w_fossa = compute_region_weights(
        full_transformed,
        condyle_reference_points,
        fossa_reference_points,
        k_neighbors=K_NEIGHBORS,
        power=WEIGHT_POWER,
        eps=EPS,
        condyle_factor=CONDYLE_WEIGHT_FACTOR,
        fossa_factor=FOSSA_WEIGHT_FACTOR
    )

    # Blend both deformation fields
    full_transformed = full_transformed + w_condyle * shift_condyle + w_fossa * shift_fossa
    print("Applied dual deformable registration to disc.")

# ============================================================
# Clean and save
# ============================================================
if dfaces is not None and len(dfaces) > 0:
    cverts, cfaces = remove_degenerate_faces(full_transformed, dfaces)
else:
    cverts, cfaces = full_transformed, None

save_mesh(out_path, cverts, cfaces)
print(f"Saved transformed disc to: {out_path}")