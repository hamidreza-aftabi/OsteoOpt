# Patient-Specific Geometry Inputs

This directory is intentionally kept empty in the public repository.

Place patient-derived and generated geometry files here before running
`../matlab/Registration_Artisynth_Main.m`. The scripts expect this directory
to contain the segmented, smoothed, remeshed, and registration-ready OBJ files
for the patient mandible, maxilla/skull, donor bone, fixation hardware, condyle
surfaces, fossa surfaces, disc, and capsule components.

Do not commit patient CT data, patient-specific meshes, generated registration
weights, or generated remeshed outputs.
