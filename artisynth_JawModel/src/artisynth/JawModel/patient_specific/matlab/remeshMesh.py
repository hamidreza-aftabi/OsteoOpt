import pymeshlab
from pathlib import Path

# Get the current directory as a Path object
current_dir = Path().resolve()

# Assume the "geometry" folder is one level up from the current directory
geometry_dir = current_dir.parent / "geometry"

# List of files to be remeshed with specific target lengths
file_list = {
    'Hollowed_Rigid_Registered_Donor_Solid_Smooth_Remehsed.obj': 0.50,
}

for file_name, target_length in file_list.items():
    # Build full input and output paths using pathlib
    input_mesh_path = geometry_dir / file_name
    output_mesh_name = f"Remeshed_{Path(file_name).stem}{Path(file_name).suffix}"
    output_mesh_path = geometry_dir / output_mesh_name

    try:
        # Create a MeshSet object
        ms = pymeshlab.MeshSet()

        # Load the mesh from the input path
        ms.load_new_mesh(str(input_mesh_path))
        print(f"Loaded mesh from {input_mesh_path}")

        # Convert the target length to a PureValue (if required by the filter)
        target_length_val = pymeshlab.PureValue(target_length)

        # Perform isotropic explicit remeshing with the specified parameters
        ms.meshing_isotropic_explicit_remeshing(
            iterations=40,
            targetlen=target_length_val,
            featuredeg=20.0,
            checksurfdist=True,
            maxsurfdist=target_length_val,
            splitflag=True,
            collapseflag=True,
            swapflag=True,
            smoothflag=True,
            reprojectflag=True
        )

        # Perform additional cleaning and repair operations
        ms.meshing_merge_close_vertices()
        ms.meshing_snap_mismatched_borders()
        ms.meshing_remove_duplicate_faces()
        ms.meshing_repair_non_manifold_edges()
        ms.meshing_repair_non_manifold_vertices()
        ms.meshing_close_holes(
            maxholesize=30,
            newfaceselected=True,
            selfintersection=True,
            refinehole=True,
            refineholeedgelen=pymeshlab.PercentageValue(2.906027)
        )

        # Save the remeshed mesh to the output path
        ms.save_current_mesh(str(output_mesh_path))
        print(f"Remeshed file has been saved to {output_mesh_path}")

    except Exception as e:
        print(f"An error occurred with file {file_name}: {e}")

    finally:
        # Ensure the MeshSet object is deleted to free resources
        del ms
        print(f"Cleaned up resources for {file_name}")
