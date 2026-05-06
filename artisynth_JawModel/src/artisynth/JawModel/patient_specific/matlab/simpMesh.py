import os
import pymeshlab
from pathlib import Path

# Get the current directory as a Path object
current_dir = Path().resolve()

# Assume the "geometry" folder is one level up from the current directory
geometry_dir = current_dir.parent / "geometry"

# Define the input and output file paths
input_mesh_path = geometry_dir / "Rigid_Registered_Donor_Solid_Smooth_Remehsed.obj"
output_mesh_path = geometry_dir / "Simplified_Rigid_Registered_Donor_Solid_Smooth_Remehsed.obj"

# Parameters for decimation
num_passes = 2         # Define the number of decimation passes to perform
targetperc = 0.5       # Each pass reduces the current face count by 50%
qualitythr = 0.3       # Quality threshold for collapsing edges (adjust as needed)
preservenormal = True  # Preserve normals during decimation
preservetopology = True  # Maintain mesh topology if possible
autoclean = True       # Clean up the mesh automatically after decimation

# Create a new MeshSet and load the mesh file
ms = pymeshlab.MeshSet()
ms.load_new_mesh(str(input_mesh_path))
print(f"Loaded mesh from: {input_mesh_path}")

# Perform the decimation the specified number of times
for i in range(num_passes):
    ms.meshing_decimation_quadric_edge_collapse(
        targetperc=targetperc,         # Reduce face count by the specified percentage
        preservenormal=preservenormal,
        preservetopology=preservetopology,
        qualitythr=qualitythr,
        autoclean=autoclean
    )
    print(f"Decimation pass {i+1} of {num_passes} completed.")

# Save the final simplified mesh
ms.save_current_mesh(str(output_mesh_path))
print(f"Simplified mesh saved to: {output_mesh_path}")
