# OsteoOpt++: Patient-Specific Optimization for Mandibular Reconstruction Planning

**Repository Status: Under Reconstruction**

OsteoOpt++ is an image-to-decision framework for mandibular reconstruction. It
uses CT-derived anatomy, virtual surgical planning, ArtiSynth simulation, and
Bayesian optimization to search reconstruction variables that improve predicted
donor-host bone union.

**Demo: one optimization iteration for the Body (B) defect case.**

https://github.com/user-attachments/assets/8453a34e-4e47-47cb-96e5-db6c425aa94e

## Workflow

![Optimization loop](assets/optimization_loop.jpg)

The optimization loop generates a candidate reconstruction, remeshes the
geometry, runs a chewing simulation, evaluates apposition and safety metrics,
and sends the cost back to the Bayesian optimizer.

## Patient-Specific Modeling

![Patient-specific modeling pipeline](assets/patient_specific_modeling.jpg)

The patient-specific layer is a preprocessing step. It registers the generic
template model to CT-derived patient anatomy, transfers muscle and ligament
attachments, updates muscle and ligament parameters, adapts the TMJ soft
tissues, and then passes the resulting digital twin to the same optimization
loop.

Main entry point:

```text
artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/Registration_Artisynth_Main.m
```

Detailed notes are in:

```text
artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/README.md
```

Patient CT data, patient meshes, generated registration JSON files, temporary
OBJs, PNGs, and MATLAB cache files are intentionally excluded from git.

## Repository Layout

- `artisynth_JawModel/` - ArtiSynth jaw model, simulation code, and MATLAB
  optimization scripts.
- `artisynth_JawModel/src/artisynth/JawModel/matlab/` - generic optimization
  entry points.
- `artisynth_JawModel/src/artisynth/JawModel/patient_specific/` -
  patient-specific model construction code.
- `artisynth_VSP/` - virtual surgical planning and reconstruction components.

## Requirements

- Windows
- JDK 8 or higher
- Eclipse IDE
- ArtiSynth Core
- MATLAB
- Python or Anaconda with `numpy`, `scipy`, `trimesh`, `open3d`, `pycpd`, and
  `pymeshlab`

Set `ARTISYNTH_HOME` to the local `artisynth_core` checkout, then in MATLAB:

```matlab
addpath(fullfile(getenv('ARTISYNTH_HOME'), 'matlab'));
setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
```

## Running

Generic optimization:

```text
artisynth_JawModel/src/artisynth/JawModel/matlab
```

Run `MainOneSegment.m`, `MainTwoSegment.m`, or the manual defect scripts
`BDefectManual.m`, `SDefectManual.m`, and `RBDefectManual.m`.

Patient-specific model construction:

```text
artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab
```

Place private geometry inputs in `../geometry`, create a local `PCSA.txt` from
`PCSA.example.txt`, and run:

```matlab
Registration_Artisynth_Main
```

## Ethics and Data Access

Patient-specific CT volumes, meshes, and derived registration outputs are not
included due to ethics and privacy constraints.

## License

This project is licensed under the GNU General Public License v3. See
[LICENSE](LICENSE) for details. Please do not redistribute the current version,
as it is intended for review purposes only.
