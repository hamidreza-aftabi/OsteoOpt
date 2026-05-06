# OsteoOpt++: Patient-Specific Optimization for Mandibular Reconstruction Planning

**Repository Status: Under Reconstruction**

OsteoOpt++ is an image-to-decision framework for mandibular reconstruction. It
combines virtual surgical planning, ArtiSynth simulation, and Bayesian
optimization to search reconstruction variables that improve predicted
donor-host bone union.

**Demo: one optimization iteration for the Body (B) defect case.**

https://github.com/user-attachments/assets/8453a34e-4e47-47cb-96e5-db6c425aa94e

## Optimization Workflow

![Optimization loop](assets/optimization_loop.jpg)

The optimization loop generates a candidate reconstruction, remeshes the
geometry, runs a chewing simulation, evaluates apposition and safety metrics,
and sends the cost back to the Bayesian optimizer.

Optimization entry points:

```text
artisynth_JawModel/src/artisynth/JawModel/matlab/MainOneSegment.m
artisynth_JawModel/src/artisynth/JawModel/matlab/MainTwoSegment.m
```

Manual single-iteration checks are available in `BDefectManual.m`,
`SDefectManual.m`, and `RBDefectManual.m`.

## Patient-Specific Modeling

![Patient-specific modeling pipeline](assets/patient_specific_modeling.jpg)

The patient-specific layer is a registration and model-construction
preprocessing step. It registers the generic template model to CT-derived
patient anatomy, transfers muscle and ligament attachments, updates muscle and
ligament parameters, adapts the TMJ soft tissues, and produces the patient
digital twin used by the optimization workflow.

Patient-specific registration entry point:

```text
artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/Registration_Artisynth_Main.m
```

## Setup

Tested on Windows.

1. Install **JDK 8 or higher** and **Eclipse IDE**.
2. Clone **ArtiSynth Core**:

   ```text
   https://github.com/artisynth/artisynth_core.git
   ```

3. In Eclipse/ArtiSynth, add the model packages and make `artisynth_VSP` and
   `artisynth_JawModel` visible on the external classpath.
4. Set `ARTISYNTH_HOME` to the local `artisynth_core` checkout. In MATLAB:

   ```matlab
   addpath(fullfile(getenv('ARTISYNTH_HOME'), 'matlab'));
   setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
   ```

5. Create an Anaconda Python environment and install the Python libraries called
   from MATLAB:

   ```bash
   conda create -n osteoopt python=3.8
   conda activate osteoopt
   pip install -r artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/requirements.txt
   ```

   ```matlab
   pyenv('Version', 'C:\path\to\anaconda3\envs\osteoopt\python.exe')
   ```

Patient-specific PCSA estimation also requires CT muscle segmentation before
running the registration/model-construction script.

## Running

For generic optimization, open MATLAB in:

```text
artisynth_JawModel/src/artisynth/JawModel/matlab
```

Run `MainOneSegment.m` for one-segment defects or `MainTwoSegment.m` for
two-segment defects.

For patient-specific registration/model construction, open MATLAB in:

```text
artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab
```

Place the required patient geometry inputs in `../geometry`, create a local
`PCSA.txt` from `PCSA.example.txt`, and run:

```matlab
Registration_Artisynth_Main
```

## Repository Layout

- `artisynth_JawModel/` - ArtiSynth jaw model, simulation code, and MATLAB
  optimization scripts.
- `artisynth_JawModel/src/artisynth/JawModel/patient_specific/` -
  patient-specific registration and model-construction scripts.
- `artisynth_VSP/` - virtual surgical planning and reconstruction components.
- `assets/` - README figures.

## Ethics and Data Access

Patient-specific CT volumes and meshes are not included due to ethics and
privacy constraints.

## License

This project is licensed under the GNU General Public License v3. See
[LICENSE](LICENSE) for details. Please do not redistribute the current version,
as it is intended for review purposes only.
