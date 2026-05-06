import numpy as np
import trimesh
from pycpd import DeformableRegistration, AffineRegistration, RigidRegistration
from pathlib import Path
import json

# ============================================================
# Flags
# ============================================================
ENABLE_RIGID = False
ENABLE_AXIS_SCALING = False
ENABLE_DEFORMABLE = True
ENABLE_TWO_STAGE_DEFORMABLE = False

# Reproducibility
np.random.seed(0)

# ============================================================
# Paths
# ============================================================
current_dir = Path().resolve()
geometry_dir = current_dir.parent / "geometry"

#source_path = str(geometry_dir / "cartilage_mandible_left2.obj")
#target_path = str(geometry_dir / "Rigid_Registered_Mandible_Solid_Smooth_Remeshed_Cartilage_left.obj")
#out_path = str(geometry_dir / "condyle_left_deformed_fixed.obj")
#params_file = str(current_dir / "registration_pipeline_condyle_left.json")

source_path = str(geometry_dir / "cartilage_skull_left.obj")
target_path = str(geometry_dir / "Rigid_Registered_Maxilla_Solid_Smooth_Remeshed_Cartilage_left.obj")
out_path = str(geometry_dir / "fossa_left_deformed_fixed.obj")
params_file = str(current_dir / "registration_pipeline_fossa_left.json")


# ============================================================
# Helper Functions
# ============================================================
def load_mesh_as_nx3(path):
    """
    Load a mesh file using trimesh and return:
      - mesh object
      - vertices as (N, 3) float64 array
    """
    mesh = trimesh.load(path, process=False)
    verts = np.asarray(mesh.vertices, dtype=np.float64)

    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError(f"Mesh '{path}' must have shape (N,3). Got {verts.shape}")

    return mesh, verts


def remove_degenerate_faces(vertices, faces):
    """
    Remove faces with repeated vertex indices, duplicate faces,
    and unreferenced vertices.
    """
    valid_faces = [f for f in faces if len(set(f)) == len(f)]
    valid_faces = np.asarray(valid_faces, dtype=np.int32)

    mesh = trimesh.Trimesh(vertices=vertices, faces=valid_faces, process=False)
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()

    return np.asarray(mesh.vertices), np.asarray(mesh.faces)


def save_mesh(path, vertices, faces):
    """
    Save mesh as OBJ. If no faces exist, save as point cloud vertices only.
    """
    if faces is None or len(faces) == 0:
        with open(path, "w") as f:
            for v in vertices:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
    else:
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        mesh.export(path)


def extract_axis_scaling_only(B):
    """
    Keep only x/y/z scaling from affine matrix B.
    Removes rotation and shear by zeroing non-diagonal terms.
    """
    return np.diag(np.diag(B))


def apply_linear_transform_full(points, B, t):
    """
    Apply affine linear transform + translation to Nx3 points.
    """
    return points @ B + t


def save_pipeline_params(file_path, rigid_params=None, axis_scaling_params=None, deformable_params=None):
    """
    Save enabled registration parameters to JSON.
    """
    params = {
        "rigid": rigid_params,
        "axis_scaling": axis_scaling_params,
        "deformable": deformable_params
    }
    with open(file_path, "w") as f:
        json.dump(params, f, indent=2)


def gaussian_kernel(X, Y, beta):
    """
    Compute Gaussian kernel matrix G(X, Y):
      G_ij = exp(-||X_i - Y_j||^2 / (2 * beta^2))
    X: (N, 3)
    Y: (M, 3)
    Returns: (N, M)
    """
    diff = X[:, None, :] - Y[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    return np.exp(-dist2 / (2.0 * beta ** 2))


def apply_deformable_cpd(points, control_points, W, beta, chunk_size=2000):
    """
    Apply learned CPD deformable transform to arbitrary Nx3 points.

    CPD deformable model:
        T(points) = points + G(points, control_points) @ W

    points:         (N, 3) points to transform
    control_points: (M, 3) source points used during CPD fitting
    W:              (M, 3) learned CPD weight matrix
    beta:           Gaussian kernel width
    """
    points = np.asarray(points, dtype=np.float64)
    control_points = np.asarray(control_points, dtype=np.float64)
    W = np.asarray(W, dtype=np.float64)

    out = np.empty_like(points)

    for start in range(0, len(points), chunk_size):
        end = min(start + chunk_size, len(points))
        chunk = points[start:end]
        G = gaussian_kernel(chunk, control_points, beta)
        out[start:end] = chunk + G @ W

    return out


class DeformableRegStorage(DeformableRegistration):
    """
    Small wrapper so learned W and beta are safely accessible after register().
    """
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


def run_deformable_stage(X, Y, alpha, beta, max_iter=200, tol=1e-5):
    """
    Run one deformable CPD stage.
    """
    reg = DeformableRegStorage(
        X=X,
        Y=Y,
        alpha=alpha,
        beta=beta,
        max_iter=max_iter,
        tol=tol
    )
    reg.register()

    if reg.W_ is None:
        raise RuntimeError("Deformable registration finished but W_ was not captured.")

    return reg


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

# If the meshes have fewer vertices than n_samples, sampling is still okay with trimesh.sample.
s_down = np.asarray(smesh.sample(n_samples), dtype=np.float64).reshape(-1, 3)
t_down = np.asarray(tmesh.sample(n_samples), dtype=np.float64).reshape(-1, 3)

# ============================================================
# Registration pipeline state
# ============================================================
full_transformed = sverts.copy()
s_down_current = s_down.copy()

rigid_params = None
axis_scaling_params = None
deformable_params = []

# ============================================================
# Rigid Registration
# ============================================================
if ENABLE_RIGID:
    rreg = RigidRegistration(X=t_down, Y=s_down_current)
    rreg.register()
    print("Rigid registration completed.")

    full_transformed = rreg.transform_point_cloud(Y=full_transformed)
    s_down_current = rreg.transform_point_cloud(Y=s_down_current)

    rigid_params = {
        "enabled": True,
        "s": float(rreg.s),
        "R": np.asarray(rreg.R, dtype=np.float64).tolist(),
        "t": np.asarray(rreg.t, dtype=np.float64).tolist()
    }

# ============================================================
# Optional Axis Scaling
# ============================================================
if ENABLE_AXIS_SCALING:
    areg = AffineRegistration(X=t_down, Y=s_down_current)
    areg.register()
    print("Affine registration completed for axis scaling extraction.")

    B_axis = extract_axis_scaling_only(areg.B)
    t_axis = np.asarray(areg.t, dtype=np.float64)

    full_transformed = apply_linear_transform_full(full_transformed, B_axis, t_axis)
    s_down_current = apply_linear_transform_full(s_down_current, B_axis, t_axis)
    print("Applied axis scaling + translation.")

    axis_scaling_params = {
        "enabled": True,
        "B": np.asarray(B_axis, dtype=np.float64).tolist(),
        "t": t_axis.tolist()
    }

# ============================================================
# Deformable Registration (coarse -> fine)
# ============================================================
if ENABLE_DEFORMABLE:
    # For smooth but still local changes:
    # - coarse stage handles broad mismatch
    # - fine stage adds more local correction
    all_deform_stages = [
        {"name": "coarse", "alpha": 2.0, "beta": 3, "max_iter": 150},
        {"name": "fine",   "alpha": 4.0, "beta": 2, "max_iter": 150},
    ]

    # If disabled, run only the first stage from the list above.
    deform_stages = all_deform_stages if ENABLE_TWO_STAGE_DEFORMABLE else all_deform_stages[:1]

    for stage in deform_stages:
        print(
            f"Running deformable stage '{stage['name']}' "
            f"(alpha={stage['alpha']}, beta={stage['beta']}, max_iter={stage['max_iter']})..."
        )

        reg = run_deformable_stage(
            X=t_down,
            Y=s_down_current,
            alpha=stage["alpha"],
            beta=stage["beta"],
            max_iter=stage["max_iter"]
        )

        # Update sampled cloud directly from CPD result
        s_down_current = reg.TY.copy()

        # Apply learned deformation field to full mesh vertices
        full_transformed = apply_deformable_cpd(
            points=full_transformed,
            control_points=reg.Y,
            W=reg.W_,
            beta=reg.beta_,
            chunk_size=2000
        )

        deformable_params.append({
            "stage": stage["name"],
            "enabled": True,
            "alpha": float(stage["alpha"]),
            "beta": float(reg.beta_),
            "max_iter": int(stage["max_iter"]),
            "Y": np.asarray(reg.Y, dtype=np.float64).tolist(),
            "W": np.asarray(reg.W_, dtype=np.float64).tolist()
        })

        print(f"CPD deformable registration completed: {stage['name']}")

if not ENABLE_RIGID and not ENABLE_AXIS_SCALING and not ENABLE_DEFORMABLE:
    print("No registration enabled. Saving original source mesh.")

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
# Save full pipeline params
# ============================================================
save_pipeline_params(
    params_file,
    rigid_params=rigid_params,
    axis_scaling_params=axis_scaling_params,
    deformable_params=deformable_params
)
print(f"Saved registration pipeline params to: {params_file}")