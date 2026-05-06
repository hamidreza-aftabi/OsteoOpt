# Patient-Specific Registration and Model Construction

Entry point:

```matlab
Registration_Artisynth_Main
```

This folder contains the patient-specific registration and model-construction
preprocessing layer for OsteoOpt++. It builds a patient-specific ArtiSynth model
once, then the resulting digital twin can be used by the existing optimization
workflow.

## What This Pipeline Does

1. Computes a maxilla-based rigid initialization.
2. Applies the rigid transform to mandible, maxilla, donor, plate, screw, and
   TMJ geometry.
3. Estimates deformable CPD fields for mandible and maxilla/skull surfaces.
4. Registers condyle/fossa regions and applies anatomy-guided disc and capsule
   adaptation.
5. Updates muscle insertion/origin landmarks, hyoid position, condyle markers,
   ligament markers, muscle length parameters, SCSA-derived muscle forces, and
   ligament rest lengths.

## Required Local Inputs

Before running, place the patient geometry inputs in:

```text
../geometry/
```

Create a local `SCSA.txt` beside these scripts using `SCSA.example.txt` as the
format. `calculateFMAX.m` converts CT-derived SCSA values to `FMAX.txt`, and
`modifyFMAX.m` applies those forces to the ArtiSynth muscles.

Use the Anaconda Python environment configured in the root README. The Python
libraries called from MATLAB are listed in `requirements.txt`, including the
PyMeshLab API and `pycpd` from `https://github.com/siavashk/pycpd`.

## Notes

- `Registration_Artisynth_Main.m` uses `ARTISYNTH_HOME`; set that environment
  variable to the local `artisynth_core` checkout before opening MATLAB.
- `modelClassName` in `Registration_Artisynth_Main.m` should point to the
  ArtiSynth model class for the patient-specific case.
- `deformableRegistration_disc.py` is the reusable CPD registration helper for
  condyle/fossa targets. Generate the needed `registration_pipeline_*.json`
  files for the relevant side before running downstream dual disc and capsule
  deformation.
