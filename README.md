# OsteoOpt++: Patient-Specific Optimization for Mandibular Reconstruction Planning with Enhanced Bone Union

[![License](https://img.shields.io/badge/license-PolyForm%20NC%201.0.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-archived-brightgreen.svg)](#)
[![Java](https://img.shields.io/badge/Java-ED8B00?logo=openjdk&logoColor=white)](https://openjdk.org/)
[![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-0076A8.svg)](https://www.mathworks.com/products/matlab.html)

OsteoOpt++ is the extended version of OsteoOpt, an image-to-decision
framework for mandibular reconstruction. It combines virtual surgical planning,
ArtiSynth simulation, and Bayesian optimization to search reconstruction
variables that improve predicted donor-host bone union.

Extended Journal version: [*Medical Image Analysis*](https://www.sciencedirect.com/science/article/pii/S1361841526003506).
Earlier conference version: [MICCAI 2025](https://www.researchgate.net/publication/395706192_OsteoOpt_A_Bayesian_Optimization_Framework_for_Enhancing_Bone_Union_Likelihood_in_Mandibular_Reconstruction_Surgery).

## News and Highlights 🎉

* **September 30, 2026** - Received the *Medical Image Analysis* Best Paper Runner-Up Award 🥈.
* **August 17, 2026** - Extended version accepted in *Medical Image Analysis* (Impact Factor: 14).
* **September 26, 2025** - Received the MICCAI 2025 Young Scientist Award 🏆.
* **August 27, 2025** - Nominated for the MICCAI 2025 Best Paper and Young Scientist Awards.
* **May 12, 2025** - Early accepted to MICCAI 2025 (top 9% of approximately 3,700 submissions).


**Demo: one optimization iteration for the Body (B) defect case.**

https://github.com/user-attachments/assets/8453a34e-4e47-47cb-96e5-db6c425aa94e

## Repository Layout

* `artisynth_JawModel/` - ArtiSynth jaw model, simulation code, and MATLAB
  optimization scripts.
* `artisynth_JawModel/src/artisynth/JawModel/patient_specific/` -
  patient-specific registration and model-construction scripts.
* `artisynth_VSP/` - virtual surgical planning and reconstruction components.
* `assets/` - README figures.

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

## Patient-Specific Registration and Modeling

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

See the [registration README](artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/README.md) for detailed inputs and configuration.

## Setup

Tested on Windows.

1. Install **MATLAB**, **JDK 8 or higher**, and **Eclipse IDE**.

2. Clone **ArtiSynth Core**:

   ```text
   https://github.com/artisynth/artisynth_core.git
   ```

3. In Eclipse/ArtiSynth, add the model packages and make `artisynth_VSP` and
   `artisynth_JawModel` visible on the external classpath.

4. Set `ARTISYNTH_HOME` to the local `artisynth_core` checkout and configure
   the MATLAB interface. Replace the example path with your installation:

   ```matlab
   setenv('ARTISYNTH_HOME', 'C:\path\to\artisynth_core');
   addpath(fullfile(getenv('ARTISYNTH_HOME'), 'matlab'));
   setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
   ```

5. Create an Anaconda Python environment and install the Python libraries called
   from MATLAB. Run the following from the repository root:

   ```bash
   conda create -n osteoopt python=3.8
   conda activate osteoopt
   pip install -r requirements.txt
   ```

   Then configure MATLAB to use that environment:

   ```matlab
   pyenv('Version', 'C:\path\to\anaconda3\envs\osteoopt\python.exe');
   ```

6. Patient-specific registration also requires 3D Slicer with the
   TotalSegmentator extension and its dependencies initialized. These
   Slicer-specific dependencies are managed separately from the Python
   environment used by MATLAB. Set the executable path in MATLAB:

   ```matlab
   setenv('SLICER_EXECUTABLE', 'C:\path\to\Slicer.exe');
   ```

   See the
   [registration README](artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/README.md)
   for installation, input preparation, and configuration details.

## Running

Choose the reconstruction and check the corresponding settings in
[Editing model settings](#editing-model-settings) before running either
workflow.

### Generic optimization

Set the MATLAB current folder to:

```text
artisynth_JawModel/src/artisynth/JawModel/matlab
```

Run `MainOneSegment.m` for a one-segment reconstruction or `MainTwoSegment.m`
for a two-segment reconstruction. The supplied one-segment examples use B or S
defects; the two-segment example is configured for RB.

### Patient-specific registration and Modeling

Prepare the inputs described in the
[registration README](artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/README.md),
then set the MATLAB current folder to:

```text
artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab
```

Run:

```matlab
Registration_Artisynth_Main
```

The registration README describes the required CT and landmark files,
patient-specific adaptation parameters, and generated outputs. It also
explains how to select the one-segment or two-segment ArtiSynth model in
`Registration_Artisynth_Main.m`.

## Editing model settings

Keep the MATLAB, screw, geometry, and material settings consistent with the
selected reconstruction.

### Segment count and MATLAB defect settings

Set `defectType` and `trial` consistently across the files for your workflow:

| Workflow | Files containing `defectType` |
| --- | --- |
| One segment: B or S | [`MainOneSegment.m`](artisynth_JawModel/src/artisynth/JawModel/matlab/MainOneSegment.m), [`helperFunctionV4WithSobolSeq.m`](artisynth_JawModel/src/artisynth/JawModel/matlab/helperFunctionV4WithSobolSeq.m), [`runArtisynthSim.m`](artisynth_JawModel/src/artisynth/JawModel/matlab/runArtisynthSim.m) |
| Two segments: RB | [`MainTwoSegment.m`](artisynth_JawModel/src/artisynth/JawModel/matlab/MainTwoSegment.m), [`helperFunctionTwo.m`](artisynth_JawModel/src/artisynth/JawModel/matlab/helperFunctionTwo.m), [`runArtisynthSimTwo.m`](artisynth_JawModel/src/artisynth/JawModel/matlab/runArtisynthSimTwo.m) |

Each file defines its own `defectType`; changing the main script does not
update the others. The main script labels results, while the helper and
simulation functions configure optimization and reconstruction.

`runArtisynthSim.m` sets `num_segment` and `num_screws` to `1`;
`runArtisynthSimTwo.m` sets both to `2`. These scripted screw counts override
`DEFAULT_NUM_SCREWS` in Java.

### Screw settings and B/S/RB placement

In
[`PlateBuilder.java`](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_VSP/src/artisynth/VSP/reconstruction/PlateBuilder.java),
set the defect selector at the top of the class:

```java
private static final ScrewDefectType SCREW_DEFECT_TYPE = ScrewDefectType.B;
```

Choose `ScrewDefectType.B`, `ScrewDefectType.S`, or `ScrewDefectType.RB` to
match MATLAB. Direction vectors and first-screw offsets are grouped below the
selector; the second-screw position is defined separately in `buildScrews()`.

`DEFAULT_NUM_SCREWS` sets the default for manual VSP use; MATLAB workflows use
`num_screws` in the simulation functions above.

### Body and geometry files

Select the rigid-body meshes in
[`bodyList.txt`](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/geometry/bodyList.txt):

- **Optimization only:** enable the generic template-model geometry block.
- **After registration:** enable the registration-output block and disable
  the optimization-only block.

For one segment, keep `donor_mesh0` and `screw0` active and comment out
`donor_mesh1` and `screw1`. For two segments, keep all four entries active.
The simulation functions also toggle the second-segment entries. Check that
the mesh filenames match your outputs, and review the donor FEM and plate
file paths in the Java model below.

### Material properties

Edit the Java model for the selected reconstruction:

| Reconstruction | Model file |
| --- | --- |
| One segment | [`JawFemDemoOptimize.java`](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/JawFemDemoOptimize.java) |
| Two segments | [`JawFemDemoOptimizeTwoWithSafety.java`](https://github.com/hamidreza-aftabi/OsteoOpt/blob/main/artisynth_JawModel/src/artisynth/JawModel/JawFemDemoOptimizeTwoWithSafety.java) |

Parameters near the beginning of these classes include:

- **Cancellous bone:** `CancellousBoneDensity`, `CancellousBoneE`, and
  `CancellousBoneNu`.
- **Cortical bone:** `corticalBoneDensity`, `corticalBoneYoungModulus`, and
  `corticalBonePoissonRatio`.
- **Titanium:** `myTitaniumDensity` for plate/screw density, and `myTitaniumE`
  and `myTitaniumNu` for plate elasticity.
- **Donor contact:** `DEFAULT_E`, `DEFAULT_Nu`, `DEFAULT_Damping`, and
  `DEFAULT_Thickness`.

Keep the `DENSITY_TO_mmKS` and `PRESSURE_TO_mmKS` conversions. When changing
bone-density assumptions, also review `corticalAppositionDensity` and
`cancellousAppositionDensity`, which are used by the bone-union metric.

Registration does not individualize material properties; see the
[registration README](artisynth_JawModel/src/artisynth/JawModel/patient_specific/matlab/README.md).

After Java edits, rebuild the project and reload the ArtiSynth model.

## Citation

If you use OsteoOpt++, please cite the journal version:

```bibtex
@article{aftabi2026patient,
  title = {Towards Patient-Specific Optimization for Mandibular Reconstruction Planning Based on Predicted Bone-Union Propensity},
  author = {Aftabi, Hamidreza and Lloyd, John E. and Ding, Amanda and Sagl, Benedikt and Prisman, Eitan and Hodgson, Antony and Fels, Sidney},
  journal = {Medical Image Analysis},
  year = {2026},
  doi = {10.1016/j.media.2026.104281},
  url = {https://www.sciencedirect.com/science/article/pii/S1361841526003506}
}
```

Please also cite the earlier OsteoOpt MICCAI conference paper:

```bibtex
@inproceedings{aftabi2025osteoopt,
  title = {{OsteoOpt}: A Bayesian Optimization Framework for Enhancing Bone Union Likelihood in Mandibular Reconstruction Surgery},
  author = {Aftabi, Hamidreza and Lloyd, John E. and Ding, Amanda and Sagl, Benedikt and Prisman, Eitan and Hodgson, Antony and Fels, Sidney},
  booktitle = {International Conference on Medical Image Computing and Computer-Assisted Intervention},
  pages = {448--458},
  year = {2025},
  organization = {Springer}
}
```

## License

OsteoOpt++ is source-available for noncommercial research and educational use
under the [PolyForm Noncommercial License 1.0.0](LICENSE). Commercial use
requires separate written permission from the authors. See [NOTICE](NOTICE) for
the required copyright notice.

## Ethics and Data Access

Patient-specific CT volumes and meshes are not included due to ethics and
privacy constraints.

Parameter values in this repository may differ from those used in the
publications because the codebase continues to support ongoing experiments.
