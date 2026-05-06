import numpy as np
import trimesh
from pathlib import Path
import json

# -------------------------
# Setup file paths
# -------------------------
current_dir = Path().resolve()
geometry_dir = current_dir.parent / "geometry"

# -------------------------
# Load the saved registration transform
# -------------------------
weights_file_path = str(current_dir / "registration_weights.json")

with open(weights_file_path, "r") as f:
    registration_weights = json.load(f)

T = np.asarray(registration_weights["transform_4x4"], dtype=np.float64)

if T.shape != (4, 4):
    raise ValueError(f"transform_4x4 must have shape (4,4). Got {T.shape}")

# -------------------------
# List of Secondary Objects
# -------------------------
secondary_objects = [
    str(geometry_dir / "Mandible_Solid_Smooth_Remeshed_With_Cartilage.obj"),
    str(geometry_dir / "Maxilla_Solid_Smooth_Remeshed_With_Cartilage.obj"),
    str(geometry_dir / "Mandible_Solid_Smooth_Remeshed_Cartilage_Left.obj"),
    str(geometry_dir / "Mandible_Solid_Smooth_Remeshed_Cartilage_Right.obj"),
    str(geometry_dir / "Mandible_Solid_Smooth_remeshed_Condyle_Left.obj"),
    str(geometry_dir / "Mandible_Solid_Smooth_remeshed_Condyle_Right.obj"),
    str(geometry_dir / "Mandible_Solid_Smooth_Remeshed_With_Cartilage_Resected_Left.obj"),
    str(geometry_dir / "Mandible_Solid_Smooth_Remeshed_With_Cartilage_Resected_Right.obj"),
    str(geometry_dir / "Maxilla_Solid_Smooth_Remeshed_Cartilage_Left.obj"),
    str(geometry_dir / "Maxilla_Solid_Smooth_Remeshed_Cartilage_Right.obj"),
    str(geometry_dir / "Donor_Solid_Smooth_Remehsed.obj"),
    str(geometry_dir / "Plate_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Screw0_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Screw1_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Screw2_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Screw3_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Screw4_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Screw5_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Screw6_Solid_Smooth_Remeshed.obj"),
    str(geometry_dir / "Disc_Solid_Smooth_Remeshed_Left.obj"),
    str(geometry_dir / "Disc_Solid_Smooth_Remeshed_Right.obj")
]


# -------------------------
# Helper Functions
# -------------------------
def apply_4x4_transform(points, T):
    """
    Apply a 4x4 rigid transform to Nx3 points.

    points' = points @ R.T + t
    """
    points = np.asarray(points, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    return points @ R.T + t


def transform_secondary_objects(secondary_objects, T):
    transformed_objects = []

    for path in secondary_objects:
        secondary_mesh = trimesh.load(path, process=False)
        secondary_verts = np.asarray(secondary_mesh.vertices, dtype=np.float64)

        transformed_verts = apply_4x4_transform(secondary_verts, T)
        transformed_objects.append((path, transformed_verts, secondary_mesh.faces))

    return transformed_objects


# -------------------------
# Transform the secondary objects
# -------------------------
transformed_secondary_verts = transform_secondary_objects(secondary_objects, T)

# -------------------------
# Save the transformed secondary objects
# -------------------------
for secondary_path, transformed_verts, faces in transformed_secondary_verts:
    secondary_name = Path(secondary_path).stem
    output_path = str(geometry_dir / f"Rigid_Registered_{secondary_name}.obj")

    secondary_transformed_mesh = trimesh.Trimesh(
        vertices=transformed_verts,
        faces=faces,
        process=False
    )
    secondary_transformed_mesh.export(output_path)

    print(f"Secondary object transformed and saved to: {output_path}")

print("All secondary objects transformed and saved successfully.")