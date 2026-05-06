import pymeshlab
from pathlib import Path

# Setup directories:
# Assumes the "geometry" folder is one level up from the current directory.
current_dir = Path().resolve()
geometry_dir = current_dir.parent / "geometry"

# Define the primary mesh filename and its path.
primary_mesh_filename = "Rigid_Registered_Donor_Solid_Smooth_Remehsed.obj"
primary_mesh_path = geometry_dir / primary_mesh_filename

# Define a list of subtractive mesh filenames.
subtractive_mesh_filenames = [
    "Rigid_Registered_Screw3_Solid_Smooth_Remeshed.obj",
    "Rigid_Registered_Screw4_Solid_Smooth_Remeshed.obj"
]

# Load the primary mesh into a MeshSet (ms_primary).
ms_primary = pymeshlab.MeshSet()
ms_primary.load_new_mesh(str(primary_mesh_path))
print(f"Loaded primary mesh: {primary_mesh_path}")

# For each subtractive mesh, perform the boolean difference in a separate MeshSet.
for sub_filename in subtractive_mesh_filenames:
    subtract_mesh_path = geometry_dir / sub_filename
    print(f"Processing subtractive mesh: {subtract_mesh_path}")
    
    # Load the subtractive mesh in its own MeshSet.
    ms_sub = pymeshlab.MeshSet()
    ms_sub.load_new_mesh(str(subtract_mesh_path))
    
    # Create a new MeshSet to combine the current primary and subtractive meshes.
    ms_combined = pymeshlab.MeshSet()
    
    # Add the current primary mesh.
    # Using add_mesh() instead of reusing the existing MeshSet avoids potential state issues.
    ms_combined.add_mesh(ms_primary.current_mesh(), "primary")
    
    # Add the subtractive mesh.
    ms_combined.add_mesh(ms_sub.current_mesh(), "subtractive")
    
    # Apply the boolean difference filter.
    # This subtracts the mesh at index 1 ("subtractive") from the mesh at index 0 ("primary").
    ms_combined.apply_filter('generate_boolean_difference', first_mesh=0, second_mesh=1)
    print(f"Subtracted {sub_filename} from the primary mesh.")
    
    # Retrieve the updated primary mesh.
    updated_primary = ms_combined.current_mesh()
    
    # Reinitialize ms_primary with the updated primary mesh.
    ms_primary.clear()          # Clear any existing meshes.
    ms_primary.add_mesh(updated_primary)
    
# Define the output filename with "Hollowed_" prefixed.
output_mesh_filename = "Hollowed_" + primary_mesh_filename
output_mesh_path = geometry_dir / output_mesh_filename

# Save the final resulting mesh.
ms_primary.save_current_mesh(str(output_mesh_path))
print(f"Saved the final hollowed mesh as: {output_mesh_path}")
