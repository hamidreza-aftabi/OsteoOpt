import numpy as np
import trimesh
from pathlib import Path
import open3d as o3d
import json

# -------------------------
# Setup file paths
# -------------------------
current_dir = Path().resolve()
geometry_dir = current_dir.parent / "geometry"

target_path = str(geometry_dir / "skull_with_cartilage.obj")
source_path = str(geometry_dir / "Maxilla_Solid_Smooth_Remeshed_With_Cartilage.obj")

# target_path = str(geometry_dir / "mandible_with_cartilage.obj")
# source_path = str(geometry_dir / "Mandible_Solid_Smooth_Remeshed_With_Cartilage.obj")

weights_file_path = str(current_dir / "registration_weights.json")


# -------------------------
# Helper functions
# -------------------------
def load_mesh_vertices(path):
    mesh = trimesh.load(path, process=False)
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError(f"Mesh '{path}' must have shape (N,3). Got {verts.shape}")
    return mesh, verts


def compute_centroids_translation(source_points, target_points):
    source_centroid = np.mean(source_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)
    return target_centroid - source_centroid


def apply_4x4_transform(points, T):
    points = np.asarray(points, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    return points @ R.T + t


def make_point_cloud(points):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    return pc


# -------------------------
# Load meshes and extract vertices
# -------------------------
smesh, sverts = load_mesh_vertices(source_path)
tmesh, tverts = load_mesh_vertices(target_path)

# Downsample to a maximum of 5000 points for efficiency
n_samples = min(len(sverts), len(tverts), 5000)
sverts_downsampled = trimesh.sample.sample_surface_even(smesh, n_samples)[0]
tverts_downsampled = trimesh.sample.sample_surface_even(tmesh, n_samples)[0]

# -------------------------
# Initial centroid alignment
# -------------------------
translation_vector = compute_centroids_translation(sverts_downsampled, tverts_downsampled)
print("Initial Translation Vector:", translation_vector)

T_init = np.eye(4, dtype=np.float64)
T_init[:3, 3] = translation_vector

# Apply initial transform to source sample
sverts_init = apply_4x4_transform(sverts_downsampled, T_init)

# -------------------------
# ICP refinement
# -------------------------
source_pc = make_point_cloud(sverts_init)
target_pc = make_point_cloud(tverts_downsampled)

threshold = 5.0

result_icp = o3d.pipelines.registration.registration_icp(
    source_pc,
    target_pc,
    threshold,
    np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

print("\nICP Transformation Matrix:")
print(result_icp.transformation)

# Total transform from original source -> target
T_total = result_icp.transformation @ T_init

print("\nTotal Transformation Matrix:")
print(T_total)

# -------------------------
# Save full rigid transform
# -------------------------
registration_weights = {
    "transform_4x4": T_total.tolist()
}

with open(weights_file_path, "w") as f:
    json.dump(registration_weights, f, indent=4)

print("Registration weights saved to:", weights_file_path)