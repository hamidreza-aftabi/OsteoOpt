# Patient-Specific Registration

This pipeline registers patient geometry, adapts both TMJs, and updates the
ArtiSynth jaw-model muscles and ligaments.

## Inputs

Run the pipeline from:

```text
artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/
```

Place these files in `inputs/`:

- `CT.nrrd`
- `SCSA_landmarks.json`

`SCSA_landmarks.json` must use RAS or LPS coordinates in millimetres and contain:

- `F_2-1`, `F_2-2`, `F_2-3`: three fiducial points defining the Frankfort
  horizontal plane.
- `Mandible Angle-1`, `Mandible Angle-2`: left and right mandibular-angle
  points.
- `Lateral Pole-1`, `Lateral Pole-2`: left and right condylar lateral-pole
  points.

The following files are read from `../../geometry/`:

```text
skull_with_cartilage.obj
mandible_with_cartilage.obj
cartilage_skull_left.obj
cartilage_skull_right.obj
cartilage_mandible_left.obj
cartilage_mandible_right.obj
disc_left.obj
disc_right.obj
caps_l_v11.obj
caps_r_v19.obj
Maxilla_Solid_Smooth_Remeshed_With_Cartilage.obj
Mandible_Solid_Smooth_Remeshed_With_Cartilage.obj
Maxilla_Solid_Smooth_Remeshed_Cartilage_Left.obj
Maxilla_Solid_Smooth_Remeshed_Cartilage_Right.obj
Mandible_Solid_Smooth_Remeshed_Cartilage_Left.obj
Mandible_Solid_Smooth_Remeshed_Cartilage_Right.obj
Mandible_Solid_Smooth_remeshed_Condyle_Left.obj
Mandible_Solid_Smooth_remeshed_Condyle_Right.obj
resected_mandible_l_opt_remeshed.obj
resected_mandible_r_opt_remeshed.obj
donor_opt0_remeshed.obj
plate_opt.obj
screw_opt0_remeshed.obj
muscleList.txt
muscleInfo.txt
closerMuscleList.txt
```

## SCSA

`generateSCSA.py` runs TotalSegmentator `head_muscles` through headless 3D
Slicer and calculates bilateral temporalis, masseter, medial-pterygoid, and
lateral-pterygoid SCSA.

- Ma/MP: 30 degrees to the Frankfort horizontal plane and translated 25 mm
  anterosuperiorly.
- Temporalis: translated 10 mm superior to the Frankfort horizontal plane.
- Lateral pterygoid: perpendicular to the Frankfort horizontal plane and
  translated 10 mm anteriorly.

Neighboring-plane sampling is off by default:

```matlab
useNeighboringScsPlanes = false;
```

Set it to `true` to measure the reference plane and ten parallel planes at
1--5 mm on each side and retain the largest area. `SCSA.txt` is written in cm²
in this order:

```text
T_R, T_L, Ma_R, Ma_L, MP_R, MP_L, LP_R, LP_L
```

## Run

Set `ARTISYNTH_HOME` and `SLICER_EXECUTABLE`, set this folder as the MATLAB
current folder, and run:

```matlab
Registration_Artisynth_Main
```

The pipeline performs rigid and deformable registration, bilateral condyle and
fossa registration, dual deformation of both discs and capsules, donor
remeshing, and patient-specific muscle and ligament updates.

Default model:

```matlab
ah1 = artisynth('-model', 'artisynth.JawModel.JawFemDemoOptimize');
```

Two-segment model:

```matlab
ah1 = artisynth('-model', 'artisynth.JawModel.JawFemDemoOptimizeTwoWithSafety');
```

## Outputs

Registered meshes are written to `../../geometry/`. Main local outputs include:

```text
SCSA.txt
WPCSA.txt
BPCSA.txt
Final_PCSA.txt
FMAX.txt
```
