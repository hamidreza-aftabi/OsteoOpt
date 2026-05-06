import numpy as np
import trimesh
from pycpd import DeformableRegistration
from pathlib import Path
import json

# ============================================================
# Paths
# ============================================================
current_dir = Path().resolve()
geometry_dir = current_dir.parent / "geometry"

source_path = geometry_dir / "skull_with_cartilage.obj"
target_path = geometry_dir / "Rigid_Registered_Maxilla_Solid_Smooth_Remeshed_With_Cartilage.obj"
out_path = geometry_dir / "Deformable_Registered_Maxilla_Solid_Smooth_Remeshed_With_Cartilage.obj"
params_file = str(current_dir / "deformation_weights_maxilla.json")

# Reproducibility
np.random.seed(0)

# ============================================================
# Helper Functions
# ============================================================
def load_mesh_as_nx3(path):
    """
    Load a mesh file using trimesh, returning the mesh object and Nx3 vertices.
    Raises ValueError if vertices aren't Nx3.
    """
    mesh = trimesh.load(path, process=False)
    verts = np.asarray(mesh.vertices, dtype=np.float64)

    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError(f"Mesh '{path}' must have shape (N,3). Got {verts.shape}")

    return mesh, verts


def remove_degenerate_faces(vertices, faces):
    """
    Remove faces with repeated vertex indices, plus unreferenced/duplicate geometry.
    Returns cleaned (vertices, faces).
    """
    valid_faces = [f for f in faces if len(set(f)) == len(f)]
    valid_faces = np.asarray(valid_faces, dtype=np.int32)

    mesh = trimesh.Trimesh(vertices=vertices, faces=valid_faces, process=False)
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()

    return np.asarray(mesh.vertices), np.asarray(mesh.faces)


def save_mesh(path, vertices, faces):
    """
    Save a mesh as a .obj file.
    If no faces are present, save as a point cloud.
    """
    if faces is None or len(faces) == 0:
        with open(path, "w") as f:
            for v in vertices:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
    else:
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        mesh.export(path)


def save_deformation_weights(Y, W, beta, file_path):
    """
    Save CPD deformation parameters to a JSON file.
    """
    params = {
        "Y": np.asarray(Y, dtype=np.float64).tolist(),
        "W": np.asarray(W, dtype=np.float64).tolist(),
        "beta": float(beta)
    }
    with open(file_path, "w") as f:
        json.dump(params, f, indent=2)


def gaussian_kernel(X, Y, beta):
    """
    Compute Gaussian kernel matrix:
        G_ij = exp(-||X_i - Y_j||^2 / (2 * beta^2))
    """
    diff = X[:, None, :] - Y[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    return np.exp(-dist2 / (2.0 * beta ** 2))


def apply_cpd_transform_full(points, control_points, W, beta, chunk_size=2000):
    """
    Apply learned CPD deformable transform to arbitrary Nx3 points:
        T(points) = points + G(points, control_points) @ W
    """
    points = np.asarray(points, dtype=np.float64)
    control_points = np.asarray(control_points, dtype=np.float64)
    W = np.asarray(W, dtype=np.float64)
    beta = float(beta)

    out_pts = np.empty_like(points)
    N = points.shape[0]

    for start in range(0, N, chunk_size):
        end = min(start + chunk_size, N)
        chunk = points[start:end]

        G = gaussian_kernel(chunk, control_points, beta)
        out_pts[start:end] = chunk + G @ W

    return out_pts


# ============================================================
# CPD wrapper to retain learned parameters
# ============================================================
class DeformableRegStorage(DeformableRegistration):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.W_ = None
        self.beta_ = None

    def update_transform(self):
        super().update_transform()
        if hasattr(self, "W"):
            self.W_ = self.W.copy()
        if hasattr(self, "beta"):
            self.beta_ = float(self.beta)


# ============================================================
# Load Source and Target Meshes
# ============================================================
smesh, sverts = load_mesh_as_nx3(source_path)
tmesh, tverts = load_mesh_as_nx3(target_path)
sfaces = np.asarray(smesh.faces) if smesh.faces is not None and len(smesh.faces) > 0 else None


# ============================================================
# Downsample for CPD Registration
# ============================================================
n_samples = 5000
s_down = np.asarray(smesh.sample(n_samples), dtype=np.float64).reshape(-1, 3)
t_down = np.asarray(tmesh.sample(n_samples), dtype=np.float64).reshape(-1, 3)


# ============================================================
# Run deformable CPD
# ============================================================
reg = DeformableRegStorage(
    X=t_down,
    Y=s_down,
    alpha=.5,
    beta=8.0,
    max_iter=150
)
reg.register()
print("CPD registration completed.")

if reg.W_ is None:
    raise RuntimeError("Registration completed but W_ was not captured.")


# ============================================================
# Transform the full source mesh
# ============================================================
full_transformed = apply_cpd_transform_full(
    points=sverts,
    control_points=reg.Y,
    W=reg.W_,
    beta=reg.beta_,
    chunk_size=2000
)


# ============================================================
# Remove degenerate faces and save final transformed mesh
# ============================================================
if sfaces is not None and len(sfaces) > 0:
    cverts, cfaces = remove_degenerate_faces(full_transformed, sfaces)
else:
    cverts, cfaces = full_transformed, None

save_mesh(out_path, cverts, cfaces)
print(f"Saved transformed mesh to: {out_path}")


# ============================================================
# Save deformation parameters
# ============================================================
save_deformation_weights(reg.Y, reg.W_, reg.beta_, params_file)
print(f"Saved deformation weights to: {params_file}")
