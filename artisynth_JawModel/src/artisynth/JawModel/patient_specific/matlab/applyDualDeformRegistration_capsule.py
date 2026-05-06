#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


# ============================================================
# Fixed configuration (edit here only)
# ============================================================
# Allowed: "left" or "right"
SIDE = "right"

CURRENT_DIR = Path(__file__).resolve().parent
GEOMETRY_DIR = CURRENT_DIR.parent / "geometry"

# File paths (edit here)
if SIDE.lower() == "right":
    INPUT_OBJ_PATH = GEOMETRY_DIR / "caps_r_v19.obj"
    CONDYLE_JSON_PATH = CURRENT_DIR / "registration_pipeline_condyle_right.json"
    FOSSA_JSON_PATH = CURRENT_DIR / "registration_pipeline_fossa_right.json"
    OUTPUT_OBJ_PATH = GEOMETRY_DIR / "caps_r_v19_dual_deformed.obj"
elif SIDE.lower() == "left":
    INPUT_OBJ_PATH = GEOMETRY_DIR / "caps_l_v11.obj"
    CONDYLE_JSON_PATH = CURRENT_DIR / "registration_pipeline_condyle_left.json"
    FOSSA_JSON_PATH = CURRENT_DIR / "registration_pipeline_fossa_left.json"
    OUTPUT_OBJ_PATH = GEOMETRY_DIR / "caps_l_v11_dual_deformed.obj"
else:
    raise ValueError("SIDE must be 'left' or 'right'")

# Vertex output precision
OUTPUT_DECIMALS = 6

# Hybrid shape controls
# Modes:
#   - "scaling_translation": current method (global/directional scaling + deformable translation)
#   - "deformation_only": no pre-scaling of geometry; only deformable translation is applied
#                         after applying user directional_scale_xyz to the base geometry.
# Notes:
#   - Global uniform scale controls below are active only in "scaling_translation" mode.
#   - GLOBAL_SCALE_ORIGIN is used in both modes (origin for directional scaling,
#     and also for global scaling when global scaling is active).
#   - ATTACHMENT_* settings are used in both modes.
TRANSFORM_MODE = "deformation_only"


# Global uniform scaling controls (active only in "scaling_translation" mode)
USE_GLOBAL_SCALING = True
GLOBAL_SCALE_MODE = "from_registration"
GLOBAL_UNIFORM_SCALE = 1.0
MIN_GLOBAL_SCALE = 0.85
MAX_GLOBAL_SCALE = 1.5

# Scaling origin (active in both modes for directional scaling;
# also used for global scaling in "scaling_translation" mode)
GLOBAL_SCALE_ORIGIN = "centroid"

# If True, directional scaling for target="condyle"/"fossa" uses that target's
# reference centroid as the scaling origin (stronger local expansion/shrink effect).
DIRECTIONAL_USE_TARGET_LOCAL_ORIGIN = True

ATTACHMENT_RADIUS_FACTOR = 6.0
ATTACHMENT_INFLUENCE_CUTOFF = 0.025

# Directional soft-blend distance falloff power.
# < 1.0 = slower decay (smoother carryover to opposite side)
# = 1.0 = inverse-distance
# > 1.0 = faster decay (sharper localization)
DIRECTIONAL_DISTANCE_FALLOFF_POWER = .2

# Surface-quality and anti-inversion controls
ENABLE_ANTI_INVERSION_CONTROLS = True
INITIAL_DEFORMATION_SCALE = 1.0
SCALE_REDUCTION_FACTOR = 0.5
ALLOWED_SURFACE_FLIPS = 0
MAX_BACKTRACK_ITERS = 25

# Surface-quality guardrails for downstream TetGen/FEM steps.
ENABLE_TETGEN_FEM_GUARDRAILS = True
MIN_TRIANGLE_AREA_RATIO = 0.20
MIN_EDGE_LENGTH_RATIO = 0.35
MAX_EDGE_LENGTH_RATIO = 2.50

# Optional hard cap on per-vertex displacement magnitude (same unit as mesh).
# Set to None to disable.
MAX_VERTEX_DISPLACEMENT = None
MAX_VERTEX_DISPLACEMENT_EDGE_RATIO = 0.35
DISPLACEMENT_SMOOTHING_ITERS = 6
DISPLACEMENT_SMOOTHING_LAMBDA = 0.30


# Weighting params
K_NEIGHBORS = 20
WEIGHT_POWER = 1.0
EPS = 1e-8
CHUNK_SIZE = 5000

# Manual influence adjustment
# > 1.0 = stronger influence
# < 1.0 = weaker influence
CONDYLE_WEIGHT_FACTOR = 1.0
FOSSA_WEIGHT_FACTOR = 1.0

SIDE_SETTINGS = {
    "left": {
        "initial_deformation_scale": 1.0,
        "attachment_radius_factor": ATTACHMENT_RADIUS_FACTOR,
        "max_vertex_displacement_edge_ratio": MAX_VERTEX_DISPLACEMENT_EDGE_RATIO,
        "directional_scale_xyz": [1.2, 1.0, 1.0],
        # Options: "all", "condyle", "fossa"
        "directional_scale_target": "all",
        "directional_scale_radius_factor": ATTACHMENT_RADIUS_FACTOR,
        "directional_scale_influence_cutoff": ATTACHMENT_INFLUENCE_CUTOFF,
        "directional_distance_falloff_power": DIRECTIONAL_DISTANCE_FALLOFF_POWER,
    },
    "right": {
        "initial_deformation_scale": 5.0,
        "attachment_radius_factor": ATTACHMENT_RADIUS_FACTOR,
        "max_vertex_displacement_edge_ratio": MAX_VERTEX_DISPLACEMENT_EDGE_RATIO,
        # Tune these three values to expand/shrink the right capsule per axis.
        "directional_scale_xyz": [1.5, 1.2, 1.0],
        # Options: "all", "condyle", "fossa"
        "directional_scale_target": "condyle",
        "directional_scale_radius_factor": ATTACHMENT_RADIUS_FACTOR,
        "directional_scale_influence_cutoff": ATTACHMENT_INFLUENCE_CUTOFF,
        "directional_distance_falloff_power": DIRECTIONAL_DISTANCE_FALLOFF_POWER,
    },
}


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

        if "Y" not in stage or "W" not in stage or "beta" not in stage:
            raise ValueError(
                f"Deformable stage {i} must contain 'Y', 'W', and 'beta'. "
                "Please regenerate the JSON using the revised registration code."
            )

        Y = np.asarray(stage["Y"], dtype=np.float64)
        W = np.asarray(stage["W"], dtype=np.float64)
        beta = float(stage["beta"])

        control_shift = reconstruct_control_shift(Y, W, beta)

        out_shift = np.zeros_like(current)
        n_points = current.shape[0]

        for start in range(0, n_points, chunk_size):
            end = min(start + chunk_size, n_points)
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

    For multi-stage deformable pipelines, use the last enabled stage's control points.
    """
    if deformable_params is None:
        raise ValueError("deformable_params is None")

    if isinstance(deformable_params, dict):
        if "Y" not in deformable_params:
            raise ValueError("Deformable params dict is missing 'Y'")
        return np.asarray(deformable_params["Y"], dtype=np.float64)

    if isinstance(deformable_params, list):
        enabled_stages = [stage for stage in deformable_params if stage is not None and stage.get("enabled", False)]
        if len(enabled_stages) == 0:
            raise ValueError("No enabled deformable stages found")
        if "Y" not in enabled_stages[-1]:
            raise ValueError("Last enabled deformable stage is missing 'Y'")
        return np.asarray(enabled_stages[-1]["Y"], dtype=np.float64)

    raise TypeError(f"Unsupported deformable_params type: {type(deformable_params)}")


def get_last_enabled_stage(deformable_params):
    if deformable_params is None:
        raise ValueError("deformable_params is None")

    if isinstance(deformable_params, dict):
        if not deformable_params.get("enabled", False):
            raise ValueError("Single deformable stage is not enabled")
        return deformable_params

    if isinstance(deformable_params, list):
        enabled_stages = [stage for stage in deformable_params if stage is not None and stage.get("enabled", False)]
        if len(enabled_stages) == 0:
            raise ValueError("No enabled deformable stages found")
        return enabled_stages[-1]

    raise TypeError(f"Unsupported deformable_params type: {type(deformable_params)}")


def query_mean_k_distance(tree, points, k=3):
    dists, _ = tree.query(points, k=k)
    if k == 1:
        dists = dists.reshape(-1, 1)
    return np.mean(dists, axis=1)


def compute_region_weights(
    points,
    condyle_reference_points,
    fossa_reference_points,
    k_neighbors=3,
    power=2.0,
    eps=1e-8,
    condyle_factor=1.0,
    fossa_factor=1.0,
):
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


def estimate_isotropic_scale_from_stage(stage):
    if "Y" not in stage or "W" not in stage or "beta" not in stage:
        raise ValueError("Stage must contain 'Y', 'W', and 'beta'")

    Y = np.asarray(stage["Y"], dtype=np.float64)
    control_shift = reconstruct_control_shift(Y, stage["W"], stage["beta"])
    TY = Y + control_shift

    src_center = np.mean(Y, axis=0)
    src_radius = np.linalg.norm(Y - src_center, axis=1)
    dst_radius = np.linalg.norm(TY - src_center, axis=1)

    valid = src_radius > 1e-8
    if not np.any(valid):
        return 1.0

    ratios = dst_radius[valid] / src_radius[valid]
    ratios = ratios[np.isfinite(ratios)]
    if ratios.size == 0:
        return 1.0

    return float(np.median(ratios))


def estimate_global_scale_from_registration(deformable_sets, return_raw=False):
    scales = []
    for deformable_params in deformable_sets:
        stage = get_last_enabled_stage(deformable_params)
        scales.append(estimate_isotropic_scale_from_stage(stage))

    if len(scales) == 0:
        if return_raw:
            return 1.0, 1.0
        return 1.0

    raw_scale = float(np.median(np.asarray(scales, dtype=np.float64)))
    clamped_scale = min(max(raw_scale, float(MIN_GLOBAL_SCALE)), float(MAX_GLOBAL_SCALE))
    if return_raw:
        return raw_scale, clamped_scale
    return clamped_scale


def apply_uniform_scaling(points, scale, origin_mode="centroid"):
    points = np.asarray(points, dtype=np.float64)
    scale = float(scale)
    if abs(scale - 1.0) < 1e-12:
        return points.copy()

    if origin_mode == "centroid":
        origin = np.mean(points, axis=0)
    else:
        raise ValueError(f"Unsupported GLOBAL_SCALE_ORIGIN: {origin_mode}")

    return origin + scale * (points - origin)


def apply_directional_scaling(points, scale_xyz, origin_mode="centroid", origin_point=None):
    points = np.asarray(points, dtype=np.float64)
    scale_xyz = np.asarray(scale_xyz, dtype=np.float64).reshape(3,)

    if np.all(np.abs(scale_xyz - 1.0) < 1e-12):
        return points.copy()

    if origin_point is not None:
        origin = np.asarray(origin_point, dtype=np.float64).reshape(3,)
    elif origin_mode == "centroid":
        origin = np.mean(points, axis=0)
    else:
        raise ValueError(f"Unsupported directional scale origin: {origin_mode}")

    return origin + (points - origin) * scale_xyz


def estimate_reference_radius(reference_points):
    reference_points = np.asarray(reference_points, dtype=np.float64)
    if len(reference_points) < 2:
        return 1.0

    tree = cKDTree(reference_points)
    dists, _ = tree.query(reference_points, k=2)
    nn = dists[:, 1]
    nn = nn[np.isfinite(nn)]
    if nn.size == 0:
        return 1.0
    return float(max(np.median(nn), 1e-6))


def compute_attachment_influence(points, reference_points, radius, cutoff=None):
    reference_points = np.asarray(reference_points, dtype=np.float64)
    if len(reference_points) == 0:
        return np.zeros((len(points), 1), dtype=np.float64)

    radius = float(max(radius, 1e-8))
    tree = cKDTree(reference_points)
    dists, _ = tree.query(points, k=1)
    influence = np.exp(-((dists / radius) ** 2))
    if cutoff is None:
        cutoff = ATTACHMENT_INFLUENCE_CUTOFF
    cutoff = float(max(cutoff, 0.0))
    influence[influence < cutoff] = 0.0
    return influence.reshape(-1, 1)


def apply_directional_scaling_with_influence(points, scale_xyz, influence, origin_mode="centroid", origin_point=None):
    points = np.asarray(points, dtype=np.float64)
    influence = np.asarray(influence, dtype=np.float64).reshape(-1, 1)
    influence = np.clip(influence, 0.0, 1.0)

    full_scaled = apply_directional_scaling(
        points,
        scale_xyz,
        origin_mode=origin_mode,
        origin_point=origin_point,
    )
    return points + influence * (full_scaled - points)


def compute_directional_scale_influence(
    points,
    target,
    condyle_reference_points,
    fossa_reference_points,
    radius_factor,
    cutoff,
    falloff_power,
):
    n = len(points)
    target = str(target).lower().strip()
    if target == "all":
        return np.ones((n, 1), dtype=np.float64)

    # Kept for API/config compatibility in soft-blend mode.
    _ = float(max(radius_factor, 1e-8))
    _ = float(max(cutoff, 0.0))
    falloff_power = float(max(falloff_power, 1e-6))

    condyle_tree = cKDTree(np.asarray(condyle_reference_points, dtype=np.float64))
    fossa_tree = cKDTree(np.asarray(fossa_reference_points, dtype=np.float64))
    d_condyle, _ = condyle_tree.query(points, k=1)
    d_fossa, _ = fossa_tree.query(points, k=1)

    # Pure distance-based inverse weighting (no manual floor).
    # This stays smooth and non-zero on both sides due to EPS regularization.
    inv_condyle = 1.0 / ((d_condyle + EPS) ** falloff_power)
    inv_fossa = 1.0 / ((d_fossa + EPS) ** falloff_power)
    inv_sum = inv_condyle + inv_fossa + 1e-12
    w_condyle_soft = (inv_condyle / inv_sum).reshape(-1, 1)
    w_fossa_soft = (inv_fossa / inv_sum).reshape(-1, 1)

    if target == "condyle":
        influence = w_condyle_soft
    elif target == "fossa":
        influence = w_fossa_soft
    else:
        raise ValueError("directional_scale_target must be one of: all, condyle, fossa")

    influence = np.clip(influence, 0.0, 1.0)
    return influence


def build_unique_edges(triangles):
    if triangles.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.int32)

    edge_set = set()
    for tri in triangles:
        a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
        edge_set.add(tuple(sorted((a, b))))
        edge_set.add(tuple(sorted((b, c))))
        edge_set.add(tuple(sorted((c, a))))

    edges = np.asarray(sorted(edge_set), dtype=np.int32)
    return edges


def build_vertex_adjacency(n_vertices, edges):
    adjacency = [[] for _ in range(n_vertices)]
    for a, b in edges:
        adjacency[a].append(b)
        adjacency[b].append(a)

    out = []
    for nbrs in adjacency:
        if len(nbrs) == 0:
            out.append(np.zeros((0,), dtype=np.int32))
        else:
            out.append(np.asarray(sorted(set(nbrs)), dtype=np.int32))
    return out


def compute_local_edge_scale(vertices, edges):
    n_vertices = len(vertices)
    if edges.shape[0] == 0:
        return np.ones((n_vertices,), dtype=np.float64)

    sums = np.zeros((n_vertices,), dtype=np.float64)
    counts = np.zeros((n_vertices,), dtype=np.int32)
    edge_vec = vertices[edges[:, 1]] - vertices[edges[:, 0]]
    edge_len = np.linalg.norm(edge_vec, axis=1)

    for i, (a, b) in enumerate(edges):
        length = edge_len[i]
        sums[a] += length
        sums[b] += length
        counts[a] += 1
        counts[b] += 1

    out = np.ones((n_vertices,), dtype=np.float64)
    mask = counts > 0
    out[mask] = sums[mask] / counts[mask]
    return out


def smooth_displacement_field(displacement, adjacency, n_iters=0, lam=0.0):
    out = np.asarray(displacement, dtype=np.float64).copy()
    if n_iters <= 0 or lam <= 0.0:
        return out

    lam = float(min(max(lam, 0.0), 1.0))
    for _ in range(int(n_iters)):
        next_out = out.copy()
        for i, nbrs in enumerate(adjacency):
            if nbrs.size == 0:
                continue
            nbr_mean = np.mean(out[nbrs], axis=0)
            next_out[i] = (1.0 - lam) * out[i] + lam * nbr_mean
        out = next_out
    return out


def cap_displacement_by_local_scale(displacement, local_scale, scale_ratio):
    if scale_ratio is None:
        return displacement

    scale_ratio = float(scale_ratio)
    if scale_ratio <= 0:
        return displacement

    out = displacement.copy()
    max_norm = scale_ratio * np.asarray(local_scale, dtype=np.float64)
    norms = np.linalg.norm(out, axis=1)
    mask = norms > max_norm
    valid = mask & (norms > 1e-12)
    if np.any(valid):
        out[valid] *= (max_norm[valid] / norms[valid]).reshape(-1, 1)
    return out


def compute_triangle_area_ratios(base_vertices, candidate_vertices, triangles):
    if triangles.shape[0] == 0:
        return np.ones((0,), dtype=np.float64)

    a = triangles[:, 0]
    b = triangles[:, 1]
    c = triangles[:, 2]

    base_cross = np.cross(base_vertices[b] - base_vertices[a], base_vertices[c] - base_vertices[a])
    cand_cross = np.cross(candidate_vertices[b] - candidate_vertices[a], candidate_vertices[c] - candidate_vertices[a])
    base_area2 = np.linalg.norm(base_cross, axis=1)
    cand_area2 = np.linalg.norm(cand_cross, axis=1)

    valid = base_area2 > 1e-12
    ratios = np.ones_like(base_area2)
    ratios[valid] = cand_area2[valid] / base_area2[valid]
    return ratios


def compute_edge_length_ratios(base_vertices, candidate_vertices, edges):
    if edges.shape[0] == 0:
        return np.ones((0,), dtype=np.float64)

    base_len = np.linalg.norm(base_vertices[edges[:, 1]] - base_vertices[edges[:, 0]], axis=1)
    cand_len = np.linalg.norm(candidate_vertices[edges[:, 1]] - candidate_vertices[edges[:, 0]], axis=1)

    ratios = np.ones_like(base_len)
    valid = base_len > 1e-12
    ratios[valid] = cand_len[valid] / base_len[valid]
    return ratios


def evaluate_surface_quality(base_vertices, candidate_vertices, triangles, edges):
    flips = count_surface_flips(base_vertices, candidate_vertices, triangles)
    area_ratios = compute_triangle_area_ratios(base_vertices, candidate_vertices, triangles)
    edge_ratios = compute_edge_length_ratios(base_vertices, candidate_vertices, edges)

    min_area_ratio = float(np.min(area_ratios)) if area_ratios.size > 0 else 1.0
    min_edge_ratio = float(np.min(edge_ratios)) if edge_ratios.size > 0 else 1.0
    max_edge_ratio = float(np.max(edge_ratios)) if edge_ratios.size > 0 else 1.0

    flip_ok = (not ENABLE_ANTI_INVERSION_CONTROLS) or (flips <= ALLOWED_SURFACE_FLIPS)
    min_area_ok = (not ENABLE_TETGEN_FEM_GUARDRAILS) or (min_area_ratio >= MIN_TRIANGLE_AREA_RATIO)
    min_edge_ok = (not ENABLE_TETGEN_FEM_GUARDRAILS) or (min_edge_ratio >= MIN_EDGE_LENGTH_RATIO)
    max_edge_ok = (not ENABLE_TETGEN_FEM_GUARDRAILS) or (max_edge_ratio <= MAX_EDGE_LENGTH_RATIO)

    accepted = flip_ok and min_area_ok and min_edge_ok and max_edge_ok
    return accepted, {
        "flips": flips,
        "min_area_ratio": min_area_ratio,
        "min_edge_ratio": min_edge_ratio,
        "max_edge_ratio": max_edge_ratio,
        "flip_ok": flip_ok,
        "min_area_ok": min_area_ok,
        "min_edge_ok": min_edge_ok,
        "max_edge_ok": max_edge_ok,
    }


def get_side_settings(side):
    defaults = {
        "initial_deformation_scale": INITIAL_DEFORMATION_SCALE,
        "attachment_radius_factor": ATTACHMENT_RADIUS_FACTOR,
        "max_vertex_displacement_edge_ratio": MAX_VERTEX_DISPLACEMENT_EDGE_RATIO,
        "directional_scale_xyz": [1.0, 1.0, 1.0],
        "directional_scale_target": "all",
        "directional_scale_radius_factor": ATTACHMENT_RADIUS_FACTOR,
        "directional_scale_influence_cutoff": ATTACHMENT_INFLUENCE_CUTOFF,
        "directional_distance_falloff_power": DIRECTIONAL_DISTANCE_FALLOFF_POWER,
    }
    custom = SIDE_SETTINGS.get(str(side).lower(), {})
    out = defaults.copy()
    out.update(custom)
    return out


def parse_face_vertex_indices(face_tokens):
    """
    Parse OBJ face tokens and return vertex indices as integers (OBJ-style).
    Supports: f v1 v2 v3, f v/vt v/vt v/vt, f v//vn ...
    """
    indices = []
    for tok in face_tokens:
        if not tok:
            continue
        vtok = tok.split("/")[0]
        if vtok == "":
            continue
        indices.append(int(vtok))
    return indices


def triangulate_obj_faces(raw_faces, n_vertices):
    """
    Convert OBJ polygon faces into 0-based triangle indices via fan triangulation.
    """
    triangles = []

    def to_zero_based(idx):
        # OBJ positive indexing starts at 1; negative indexing is relative to end.
        if idx > 0:
            return idx - 1
        return n_vertices + idx

    for face in raw_faces:
        if len(face) < 3:
            continue

        verts = [to_zero_based(v) for v in face]
        if any(v < 0 or v >= n_vertices for v in verts):
            continue

        root = verts[0]
        for i in range(1, len(verts) - 1):
            a, b, c = root, verts[i], verts[i + 1]
            if a != b and b != c and a != c:
                triangles.append([a, b, c])

    if len(triangles) == 0:
        return np.zeros((0, 3), dtype=np.int32)

    return np.asarray(triangles, dtype=np.int32)


def count_surface_flips(base_vertices, candidate_vertices, triangles):
    if triangles.shape[0] == 0:
        return 0

    a = triangles[:, 0]
    b = triangles[:, 1]
    c = triangles[:, 2]

    base_n = np.cross(base_vertices[b] - base_vertices[a], base_vertices[c] - base_vertices[a])
    cand_n = np.cross(candidate_vertices[b] - candidate_vertices[a], candidate_vertices[c] - candidate_vertices[a])

    base_norm = np.linalg.norm(base_n, axis=1)
    cand_norm = np.linalg.norm(cand_n, axis=1)
    valid = (base_norm > 1e-14) & (cand_norm > 1e-14)
    if not np.any(valid):
        return 0

    dots = np.einsum("ij,ij->i", base_n[valid], cand_n[valid])
    flips = np.count_nonzero(dots < 0.0)
    return int(flips)


def cap_vertex_displacement(displacement, max_norm):
    if max_norm is None:
        return displacement
    max_norm = float(max_norm)
    if max_norm <= 0:
        return displacement

    out = displacement.copy()
    norms = np.linalg.norm(out, axis=1)
    mask = norms > max_norm
    if np.any(mask):
        out[mask] *= (max_norm / norms[mask]).reshape(-1, 1)
    return out


def parse_obj_with_vertex_positions(obj_path):
    """
    Parse OBJ and return:
      - all lines in original order
      - vertex values in encountered order
      - line indices where vertex records appear

    This allows replacing only vertex coordinates while preserving all non-vertex
    lines exactly as they were in the input.
    """
    with open(obj_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    vertices = []
    vertex_line_indices = []
    raw_faces = []

    for idx, line in enumerate(lines):
        if line.startswith("v "):
            parts = line.strip().split()
            if len(parts) < 4:
                raise ValueError(f"Malformed vertex line at line {idx + 1}: {line.rstrip()}")
            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
            except ValueError as exc:
                raise ValueError(f"Invalid numeric vertex at line {idx + 1}: {line.rstrip()}") from exc

            vertices.append([x, y, z])
            vertex_line_indices.append(idx)
        elif line.startswith("f "):
            parts = line.strip().split()
            if len(parts) >= 4:
                try:
                    face_idx = parse_face_vertex_indices(parts[1:])
                except ValueError:
                    face_idx = []
                if len(face_idx) >= 3:
                    raw_faces.append(face_idx)

    if len(vertices) == 0:
        raise ValueError(f"No vertices found in OBJ: {obj_path}")

    vertices_np = np.asarray(vertices, dtype=np.float64)
    triangles = triangulate_obj_faces(raw_faces, n_vertices=vertices_np.shape[0])

    return lines, vertices_np, vertex_line_indices, triangles


def rewrite_obj_vertices_only(lines, vertex_line_indices, transformed_vertices, output_path, decimals=6):
    if len(vertex_line_indices) != len(transformed_vertices):
        raise ValueError("Vertex count mismatch while writing OBJ")

    out_lines = list(lines)

    for i, line_idx in enumerate(vertex_line_indices):
        vx, vy, vz = transformed_vertices[i]
        out_lines[line_idx] = f"v {vx:.{decimals}f} {vy:.{decimals}f} {vz:.{decimals}f}\n"

    with open(output_path, "w", encoding="utf-8") as f:
        f.writelines(out_lines)


def read_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def run_dual_capsule_deformation(
    side,
    input_obj=None,
    condyle_params_file=None,
    fossa_params_file=None,
    output_obj=None,
    decimals=6,
):
    input_obj = Path(input_obj) if input_obj is not None else INPUT_OBJ_PATH
    condyle_params_file = Path(condyle_params_file) if condyle_params_file is not None else CONDYLE_JSON_PATH
    fossa_params_file = Path(fossa_params_file) if fossa_params_file is not None else FOSSA_JSON_PATH
    output_obj = Path(output_obj) if output_obj is not None else OUTPUT_OBJ_PATH

    if not input_obj.exists():
        raise FileNotFoundError(f"Input OBJ not found: {input_obj}")
    if not condyle_params_file.exists():
        raise FileNotFoundError(f"Condyle JSON not found: {condyle_params_file}")
    if not fossa_params_file.exists():
        raise FileNotFoundError(f"Fossa JSON not found: {fossa_params_file}")

    lines, vertices, vertex_line_indices, triangles = parse_obj_with_vertex_positions(input_obj)
    n_vertices_before = len(vertices)
    edges = build_unique_edges(triangles)
    adjacency = build_vertex_adjacency(n_vertices_before, edges)
    local_edge_scale = compute_local_edge_scale(vertices, edges)

    condyle_params = read_json(condyle_params_file)
    fossa_params = read_json(fossa_params_file)

    deformable_condyle = condyle_params.get("deformable", None)
    deformable_fossa = fossa_params.get("deformable", None)
    if deformable_condyle is None:
        raise ValueError("Condyle JSON does not contain deformable parameters")
    if deformable_fossa is None:
        raise ValueError("Fossa JSON does not contain deformable parameters")

    side_settings = get_side_settings(side)
    condyle_reference_points = get_reference_points_for_weighting(deformable_condyle)
    fossa_reference_points = get_reference_points_for_weighting(deformable_fossa)
    mode = str(TRANSFORM_MODE).lower().strip()
    if mode not in {"scaling_translation", "deformation_only"}:
        raise ValueError("TRANSFORM_MODE must be 'scaling_translation' or 'deformation_only'")

    directional_scale_xyz = np.asarray(side_settings["directional_scale_xyz"], dtype=np.float64).reshape(3,)
    directional_scale_target = str(side_settings["directional_scale_target"]).lower().strip()
    directional_scale_radius_factor = float(side_settings["directional_scale_radius_factor"])
    directional_scale_influence_cutoff = float(side_settings["directional_scale_influence_cutoff"])
    directional_distance_falloff_power = float(side_settings["directional_distance_falloff_power"])
    directional_origin_point = None
    if DIRECTIONAL_USE_TARGET_LOCAL_ORIGIN:
        if directional_scale_target == "condyle":
            directional_origin_point = np.mean(np.asarray(condyle_reference_points, dtype=np.float64), axis=0)
        elif directional_scale_target == "fossa":
            directional_origin_point = np.mean(np.asarray(fossa_reference_points, dtype=np.float64), axis=0)

    global_scale_raw = 1.0
    global_scale = 1.0
    if mode == "scaling_translation":
        if USE_GLOBAL_SCALING:
            if GLOBAL_SCALE_MODE == "manual":
                global_scale_raw = float(GLOBAL_UNIFORM_SCALE)
                global_scale = float(GLOBAL_UNIFORM_SCALE)
            elif GLOBAL_SCALE_MODE == "from_registration":
                global_scale_raw, global_scale = estimate_global_scale_from_registration(
                    [deformable_condyle, deformable_fossa],
                    return_raw=True,
                )
            else:
                raise ValueError(f"Unsupported GLOBAL_SCALE_MODE: {GLOBAL_SCALE_MODE}")

        scaled_vertices = apply_uniform_scaling(vertices, global_scale, origin_mode=GLOBAL_SCALE_ORIGIN)
        directional_scale_influence = compute_directional_scale_influence(
            scaled_vertices,
            directional_scale_target,
            condyle_reference_points,
            fossa_reference_points,
            directional_scale_radius_factor,
            directional_scale_influence_cutoff,
            directional_distance_falloff_power,
        )
        predeformed_vertices = apply_directional_scaling_with_influence(
            scaled_vertices,
            directional_scale_xyz,
            directional_scale_influence,
            origin_mode=GLOBAL_SCALE_ORIGIN,
            origin_point=directional_origin_point,
        )
        displacement_directional_factor_xyz = np.ones((3,), dtype=np.float64)
    else:
        # Deformation-only mode skips global scaling but keeps user directional shaping.
        directional_scale_influence = compute_directional_scale_influence(
            vertices,
            directional_scale_target,
            condyle_reference_points,
            fossa_reference_points,
            directional_scale_radius_factor,
            directional_scale_influence_cutoff,
            directional_distance_falloff_power,
        )
        predeformed_vertices = apply_directional_scaling_with_influence(
            vertices,
            directional_scale_xyz,
            directional_scale_influence,
            origin_mode=GLOBAL_SCALE_ORIGIN,
            origin_point=directional_origin_point,
        )
        displacement_directional_factor_xyz = np.ones((3,), dtype=np.float64)

    shift_condyle = compute_normalized_transfer_shift(predeformed_vertices, deformable_condyle, chunk_size=CHUNK_SIZE)
    shift_fossa = compute_normalized_transfer_shift(predeformed_vertices, deformable_fossa, chunk_size=CHUNK_SIZE)

    w_condyle, w_fossa = compute_region_weights(
        predeformed_vertices,
        condyle_reference_points,
        fossa_reference_points,
        k_neighbors=K_NEIGHBORS,
        power=WEIGHT_POWER,
        eps=EPS,
        condyle_factor=CONDYLE_WEIGHT_FACTOR,
        fossa_factor=FOSSA_WEIGHT_FACTOR,
    )

    condyle_radius = side_settings["attachment_radius_factor"] * estimate_reference_radius(condyle_reference_points)
    fossa_radius = side_settings["attachment_radius_factor"] * estimate_reference_radius(fossa_reference_points)
    condyle_influence = compute_attachment_influence(predeformed_vertices, condyle_reference_points, condyle_radius)
    fossa_influence = compute_attachment_influence(predeformed_vertices, fossa_reference_points, fossa_radius)

    displacement = (
        (w_condyle * condyle_influence) * shift_condyle
        + (w_fossa * fossa_influence) * shift_fossa
    )
    displacement = smooth_displacement_field(
        displacement,
        adjacency,
        n_iters=DISPLACEMENT_SMOOTHING_ITERS,
        lam=DISPLACEMENT_SMOOTHING_LAMBDA,
    )
    displacement = cap_displacement_by_local_scale(
        displacement,
        local_edge_scale,
        side_settings["max_vertex_displacement_edge_ratio"],
    )
    displacement = cap_vertex_displacement(displacement, MAX_VERTEX_DISPLACEMENT)
    displacement = displacement * displacement_directional_factor_xyz.reshape(1, 3)

    scale_used = float(side_settings["initial_deformation_scale"])
    effective_scale_xyz = global_scale * directional_scale_xyz

    transformed_vertices = predeformed_vertices + scale_used * displacement

    bt = 0
    guardrail_triggered = {
        "flip": False,
        "min_area": False,
        "min_edge": False,
        "max_edge": False,
    }

    if triangles.shape[0] > 0:
        accepted, metrics = evaluate_surface_quality(vertices, transformed_vertices, triangles, edges)
        initial_metrics = dict(metrics)
        if ENABLE_ANTI_INVERSION_CONTROLS and not metrics["flip_ok"]:
            guardrail_triggered["flip"] = True
        if ENABLE_TETGEN_FEM_GUARDRAILS and not metrics["min_area_ok"]:
            guardrail_triggered["min_area"] = True
        if ENABLE_TETGEN_FEM_GUARDRAILS and not metrics["min_edge_ok"]:
            guardrail_triggered["min_edge"] = True
        if ENABLE_TETGEN_FEM_GUARDRAILS and not metrics["max_edge_ok"]:
            guardrail_triggered["max_edge"] = True

        while (not accepted) and bt < MAX_BACKTRACK_ITERS:
            scale_used *= float(SCALE_REDUCTION_FACTOR)
            transformed_vertices = predeformed_vertices + scale_used * displacement
            accepted, metrics = evaluate_surface_quality(vertices, transformed_vertices, triangles, edges)
            if ENABLE_ANTI_INVERSION_CONTROLS and not metrics["flip_ok"]:
                guardrail_triggered["flip"] = True
            if ENABLE_TETGEN_FEM_GUARDRAILS and not metrics["min_area_ok"]:
                guardrail_triggered["min_area"] = True
            if ENABLE_TETGEN_FEM_GUARDRAILS and not metrics["min_edge_ok"]:
                guardrail_triggered["min_edge"] = True
            if ENABLE_TETGEN_FEM_GUARDRAILS and not metrics["max_edge_ok"]:
                guardrail_triggered["max_edge"] = True
            bt += 1

        if not accepted:
            raise RuntimeError(
                "Could not find a surface-safe deformation after backtracking. "
                f"Last metrics: flips={metrics['flips']}, "
                f"min_area_ratio={metrics['min_area_ratio']:.4f}, "
                f"min_edge_ratio={metrics['min_edge_ratio']:.4f}, "
                f"max_edge_ratio={metrics['max_edge_ratio']:.4f}. "
                "Reduce the attachment falloff, deformation scale, or local displacement cap."
            )
    else:
        metrics = {
            "flips": 0,
            "min_area_ratio": 1.0,
            "min_edge_ratio": 1.0,
            "max_edge_ratio": 1.0,
            "flip_ok": True,
            "min_area_ok": True,
            "min_edge_ok": True,
            "max_edge_ok": True,
        }
        initial_metrics = dict(metrics)

    if len(transformed_vertices) != n_vertices_before:
        raise RuntimeError("Vertex count changed unexpectedly; refusing to write output")

    rewrite_obj_vertices_only(lines, vertex_line_indices, transformed_vertices, output_obj, decimals=decimals)

    print(f"Side: {side}")
    print(f"Input OBJ: {input_obj}")
    print(f"Condyle JSON: {condyle_params_file}")
    print(f"Fossa JSON: {fossa_params_file}")
    print(f"Output OBJ: {output_obj}")
    print(f"Vertices transformed: {n_vertices_before}")
    print(f"Transform mode: {mode}")
    if mode == "scaling_translation" and USE_GLOBAL_SCALING and GLOBAL_SCALE_MODE == "from_registration":
        print(f"Global scale raw (before clamp): {global_scale_raw:.6f}")
        print(
            "Global scale clamp range: "
            f"[{MIN_GLOBAL_SCALE:.6f}, {MAX_GLOBAL_SCALE:.6f}]"
        )
    if mode == "scaling_translation":
        print(f"Global scale used: {global_scale:.6f}")
    else:
        print("Global scale used: 1.000000 (ignored in deformation_only mode)")
    print(f"Applied deformation scale: {scale_used:.6f}")
    print(
        "Side safety settings: "
        f"attachment_radius_factor={side_settings['attachment_radius_factor']:.4f}, "
        f"max_vertex_displacement_edge_ratio={side_settings['max_vertex_displacement_edge_ratio']:.4f}"
    )
    print(
        "Directional scale xyz: "
        f"[{directional_scale_xyz[0]:.4f}, {directional_scale_xyz[1]:.4f}, {directional_scale_xyz[2]:.4f}]"
    )
    print(
        "Directional scale target: "
        f"target={directional_scale_target}, "
        f"radius_factor={directional_scale_radius_factor:.4f}, "
        f"cutoff={directional_scale_influence_cutoff:.4f}, "
        f"falloff_power={directional_distance_falloff_power:.4f}"
    )
    if directional_origin_point is not None:
        print(
            "Directional scale origin: "
            f"target_local_centroid=[{directional_origin_point[0]:.4f}, "
            f"{directional_origin_point[1]:.4f}, {directional_origin_point[2]:.4f}]"
        )
    else:
        print(f"Directional scale origin: {GLOBAL_SCALE_ORIGIN}")
    print(
        "Directional scale influence stats: "
        f"min={float(np.min(directional_scale_influence)):.4f}, "
        f"mean={float(np.mean(directional_scale_influence)):.4f}, "
        f"max={float(np.max(directional_scale_influence)):.4f}"
    )
    if mode == "scaling_translation":
        print(
            "Pre-deformation total scale xyz (global * directional): "
            f"[{effective_scale_xyz[0]:.4f}, {effective_scale_xyz[1]:.4f}, {effective_scale_xyz[2]:.4f}]"
        )
    else:
        print(
            "Pre-deformation directional scale xyz: "
            f"[{directional_scale_xyz[0]:.4f}, {directional_scale_xyz[1]:.4f}, {directional_scale_xyz[2]:.4f}]"
        )
    print(
        "Surface metrics: "
        f"flips={metrics['flips']}, "
        f"min_area_ratio={metrics['min_area_ratio']:.4f}, "
        f"min_edge_ratio={metrics['min_edge_ratio']:.4f}, "
        f"max_edge_ratio={metrics['max_edge_ratio']:.4f}"
    )
    print(
        "Surface groups enabled: "
        f"anti_inversion={ENABLE_ANTI_INVERSION_CONTROLS}, "
        f"tetgen_fem_guardrails={ENABLE_TETGEN_FEM_GUARDRAILS}"
    )
    print(
        "Surface checks enabled: "
        f"flip={ENABLE_ANTI_INVERSION_CONTROLS}, "
        f"min_area={ENABLE_TETGEN_FEM_GUARDRAILS}, "
        f"min_edge={ENABLE_TETGEN_FEM_GUARDRAILS}, "
        f"max_edge={ENABLE_TETGEN_FEM_GUARDRAILS}"
    )
    print(
        "Surface checks pass: "
        f"flip_ok={metrics['flip_ok']}, "
        f"min_area_ok={metrics['min_area_ok']}, "
        f"min_edge_ok={metrics['min_edge_ok']}, "
        f"max_edge_ok={metrics['max_edge_ok']}"
    )
    print(
        "Surface checks pass (initial attempt): "
        f"flip_ok={initial_metrics['flip_ok']}, "
        f"min_area_ok={initial_metrics['min_area_ok']}, "
        f"min_edge_ok={initial_metrics['min_edge_ok']}, "
        f"max_edge_ok={initial_metrics['max_edge_ok']}"
    )
    print(f"Backtracking iterations used: {bt}")
    print(
        "Guardrail triggered this run: "
        f"flip={guardrail_triggered['flip']}, "
        f"min_area={guardrail_triggered['min_area']}, "
        f"min_edge={guardrail_triggered['min_edge']}, "
        f"max_edge={guardrail_triggered['max_edge']}, "
        f"any={any(guardrail_triggered.values())}"
    )
    print("Dual deformable transformation applied with vertex/order preservation.")


def main():
    side = SIDE.lower()
    if side not in {"left", "right"}:
        raise ValueError("SIDE must be 'left' or 'right'")

    run_dual_capsule_deformation(
        side=side,
        input_obj=INPUT_OBJ_PATH,
        condyle_params_file=CONDYLE_JSON_PATH,
        fossa_params_file=FOSSA_JSON_PATH,
        output_obj=OUTPUT_OBJ_PATH,
        decimals=OUTPUT_DECIMALS,
    )


if __name__ == "__main__":
    main()
