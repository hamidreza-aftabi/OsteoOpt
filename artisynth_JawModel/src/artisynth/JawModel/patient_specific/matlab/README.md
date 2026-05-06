# Patient-Specific Modeling Code

Entry point:

```matlab
Registration_Artisynth_Main
```

This folder contains the reusable patient-specific preprocessing layer for
OsteoOpt++. It builds a patient-specific ArtiSynth model once, then the
resulting model can be passed into the same planning, remeshing, simulation,
and Bayesian optimization workflow used by the generic cases.

## What This Pipeline Does

1. Computes a maxilla-based rigid initialization.
2. Applies the rigid transform to mandible, maxilla, donor, plate, screw, and
   TMJ geometry.
3. Estimates deformable CPD fields for mandible and maxilla/skull surfaces.
4. Registers condyle/fossa regions and applies anatomy-guided disc and optional
   capsule adaptation.
5. Updates muscle insertion/origin landmarks, hyoid position, condyle markers,
   ligament markers, muscle length parameters, PCSA-derived muscle forces, and
   ligament rest lengths.

## Inputs Kept Out Of Git

Patient-specific CT data, meshes, registration weights, generated JSON files,
temporary OBJ files, visualization PNGs, and MATLAB build caches are excluded by
`.gitignore`.

Before running, place patient geometry in:

```text
../geometry/
```

Create a local `PCSA.txt` beside these scripts using `PCSA.example.txt` as the
format. The values should come from the CT-derived PCSA estimation step.

## Notes

- `Registration_Artisynth_Main.m` uses `ARTISYNTH_HOME`; set that environment
  variable to the local `artisynth_core` checkout before opening MATLAB.
- `modelClassName` in `Registration_Artisynth_Main.m` should point to the
  ArtiSynth model class for the patient-specific case.
- `deformableRegistration_disc.py` is the reusable CPD registration helper for
  condyle/fossa targets. Generate the needed `registration_pipeline_*.json`
  files for the relevant side before enabling downstream dual disc or capsule
  deformation.
- Generated files such as `registration_weights.json`,
  `deformation_weights_*.json`, `registration_pipeline_*.json`,
  `muscle_points.txt`, and `closest_vertex.txt` are intermediate outputs and
  should not be committed.
