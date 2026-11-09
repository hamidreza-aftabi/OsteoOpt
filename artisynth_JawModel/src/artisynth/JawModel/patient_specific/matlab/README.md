# Patient-Specific Registration

This pipeline registers patient geometry, adapts both TMJs, and updates the
ArtiSynth jaw-model muscles and ligaments.

## Software setup

The workflow uses two separate Python environments:

1. **MATLAB Python environment:** used by the registration and mesh-processing
   scripts called with `pyrunfile`. From the repository root, install:

   ```bash
   pip install -r requirements.txt
   ```

   Configure MATLAB to use this environment with `pyenv`.

2. **3D Slicer Python environment:** used only by `generateSCSA.py`. Install 3D
   Slicer, install the TotalSegmentator extension from the Extension Manager,
   accept its extension dependencies, and initialize its Python dependencies
   once when prompted. PyTorch, nnUNet, TotalSegmentator, `slicer`, and VTK are
   provided inside Slicer and should not be added to the repository
   `requirements.txt`.

   This is the recommended and easier setup because the TotalSegmentator
   extension installs and manages its required dependencies. A fully
   independent PyTorch environment is possible, but it would require a separate
   non-Slicer implementation of `generateSCSA.py`.

Set the required paths before running MATLAB:

```matlab
setenv('ARTISYNTH_HOME', 'C:\path\to\artisynth_core');
setenv('SLICER_EXECUTABLE', 'C:\path\to\Slicer.exe');
```

After this one-time setup, MATLAB runs `generateSCSA.py` through Slicer in
headless mode.

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


## Patient-Specific Deformable Registration and TMJ Adaptation

The deformable registration and TMJ disc/capsule adaptation parameters should be fine-tuned for each patient based on their anatomical geometry.

1. **CPD Registration:** In [deformableRegistration_disc.py](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/deformableRegistration_disc.py), the condyle-side and fossa-side registration parameters (`alpha`, `beta`, and `max_iter`) can be adjusted independently for the left and right TMJs. The default settings are:

   ```python
   "condyle": {"alpha": 2.0, "beta": 3.0, "max_iter": 150},
   "fossa": {"alpha": 2.0, "beta": 3.0, "max_iter": 150},
   ```

   Here, `alpha` controls deformation regularization, `beta` controls the Gaussian kernel width, and `max_iter` sets the maximum number of iterations. Refer to the Coherent Point Drift (CPD) algorithm for details.

2. **Dual Disc Deformation:** In [applyDualDeformRegistration_disc.py](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/applyDualDeformRegistration_disc.py), `k_neighbors`, `weight_power`, `condyle_weight_factor`, and `fossa_weight_factor` control the spatial blending of condyle-side and fossa-side deformation fields. These parameters should be tuned for each disc to achieve appropriate deformation based on patient-specific TMJ geometry.

3. **Dual Capsule Deformation:** In [applyDualDeformRegistration_capsule.py](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/applyDualDeformRegistration_capsule.py), `K_NEIGHBORS`, `WEIGHT_POWER`, and the condyle/fossa weight factors control deformation blending. Additional parameters, including `attachment_radius_factor`, `directional_distance_falloff_power`, `directional_scale_xyz`, `directional_scale_target`, and `initial_deformation_scale`, control the spatial influence, scaling, and magnitude of capsule deformation. Displacement limits and surface-quality safeguards should also be considered when tuning these parameters for each patient.

Similarly, the CPD `alpha` and `beta` parameters in [deformableRegistration_mand.py](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/deformableRegistration_mand.py) and [deformableRegistration_skull.py](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/deformableRegistration_skull.py) should be adjusted for each patient to improve registration accuracy and muscle attachment positioning in the registered model.


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

**Important Notes:**
1. **Material Properties:** The material properties used in the patient-specific registration pipeline are inherited from the generic ArtiSynth jaw model and are not individualized for each patient, as this pipeline is intended solely for demonstration purposes. Patient-specific material properties would require estimation based on each patient's CT data, as reported in the paper.

2. **Muscle and Ligament Removal:** The functions `removeRBMusclesPatient1` and `removeLigamentsPatient1` in `Registration_Artisynth_Main.m` are examples of muscle and ligament removal for a patient with an RB (ramus-body) scapular reconstruction defect. These functions can be adjusted for different patients based on their specific defect configurations and anatomical conditions and are provided for demonstration purposes only.
