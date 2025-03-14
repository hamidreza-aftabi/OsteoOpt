# OsteoOpt: A Bayesian Optimization Framework for Enhancing Bone Union Likelihood in Mandibular Reconstruction Surgery

**Paper Submitted to MICCAI 2025**  
**Repository Status: Under Reconstruction**

---

## 1. Overview

**OsteoOpt** is a Bayesian optimization framework designed to improve bone union likelihood in mandibular reconstruction surgery and facilitates computer-aided intervention by systematically varying key surgical parameters—resection plane orientation, donor bone positioning, and graft length—across three mandibular regions. This repository contains the core code and configuration details required to set up the system. 

**Demo illustrating a single iteration of the optimization process for the Body (B) defect case (video speed increased for better demonstration:**


https://github.com/user-attachments/assets/674e7652-5fcb-4568-b28b-76c239419254


---



## 2. Prerequisites

Before installing and running the framework, ensure your system meets the following requirements (tested on Windows 10; currently only compatible with Windows):

- **[JDK 8 or Higher](https://www.oracle.com/java/technologies/downloads/) and [Eclipse IDE](https://eclipseide.org/)**  
  
- **[Artisynth](https://www.artisynth.org/Main/HomePage) Components:**  
  - Artisynth Core  
  - Artisynth Model  
  - Artisynth VSP  
  - Artisynth Jaw Model  
  *(Note: Artisynth VSP and Artisynth Jaw Model are included within this repository.)*

- **[MATLAB](https://www.mathworks.com/products/matlab.html):**  
  Version 2023a is recommended. Ensure that MATLAB is properly configured to interface with the installed JDK and Python environment.

- **Python 3.8 (via [Anaconda](https://www.anaconda.com/download)):**  
  Use Python 3.8 or a compatible version, ensuring it matches your MATLAB integration requirements.

- **[PyMeshLab](https://pymeshlab.readthedocs.io/en/latest/installation.html):**  
  Install within your Python environment (e.g., by running `pip3 install pymeshlab`).

---

## 3. Installation Instructions

### 3.1 Java and Eclipse Setup

1. **Install JDK 8 or Higher (64-bit):**  
   ArtiSynth requires a full 64-bit Java Development Kit (JDK) with a Java compiler—using only a Java Runtime Environment (JRE) is not sufficient.  
   - **Download:** We recommend installing a JDK from Oracle. Visit [Oracle Java Downloads](https://www.oracle.com/java/technologies/downloads/) and choose the appropriate installer for your system. For Windows, the easiest option is often the “x64 Installer.”  
   - **Note for JDK Versions:**  
     - For ARM-based Windows systems, you must still install a 64-bit Intel-based JDK (look for “x64” in the download name) to run via the Intel compatibility layer.

2. **Verify the JDK Installation:**  
   Open a CMD window and run:
   ```bash
   javac -version

3. **Install Eclipse:**
   - Donwload and install [Eclipse IDE](https://eclipseide.org/)
   - Open **Window > Preferences** from the main menu.
   - In the Preferences dialog, go to **Java > Installed JREs** and click Add....
   - In the JRE Type dialog, keep **Standard VM** selected and click Next.
   - In the JRE Definition dialog, enter the JDK installation folder

### 3.2 Artisynth Components

4. **Download Required Repositories:** Open Eclipse and import the GitHub project using **File → Import → Projects from Git → Clone URI**.

   - **Artisynth Core:**  
     The current development version of `artisynth_core` is available from GitHub. To clone it, use the following URL in Eclipse:
     ```bash
     https://github.com/artisynth/artisynth_core.git
     ```

   - **Artisynth Models:**  
     The current development version of `artisynth_models` is available from GitHub. To clone it, use the following URL in Eclipse:
     ```bash
     https://github.com/artisynth/artisynth_models.git
     ```

   - **Artisynth VSP & Artisynth Jaw Model:**  
     These components are included in this repository. You can find them in the `artisynth_VSP` and `artisynth_JawModel` directories.

For more information on additional details, visit [Artisynth Webpage](https://www.artisynth.org/Software/ModelsDownload).


5. **Configure Projects:**  
   - Set the run configuration for `artisynth_core` so that it has access to the three supporting libraries.
   - Adjust the build settings for each supporting library to ensure they are visible during runtime.
   - Launch the `artisynth_core` application and add the models by selecting **Models -> Edit Menu -> Add Packages**.
   - Then, go to **Settings -> External Classpath -> Add Class Folder** and add the folders for `artisynth_VSP` and `artisynth_JawModel` separately to ensure they are visible externally through MATLAB.

### 3.3 MATLAB Integration

6. **Set External Class Path:**  
    Install and open MATLAB. Then add the `artisynth_core` MATLAB folder to your path so that Java classes are available. For example:
  
```matlab
addpath(fullfile('path','to','artisynth_core','matlab'));
```

7. **Environment Variable:**  
 - Set the `ARTISYNTH_HOME` environment variable to the path where `artisynth_core` is installed. Then set the Artisynth class path in MATLAB using:

```matlab
setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
```

### 3.4 Python Environment Setup

8. **Install Python via Anaconda:**  
   Create and activate a Python 3.8 environment (or a compatible version) using the following commands:
   ```bash
   conda create -n matlab_env python=3.8
   conda activate matlab_env

9. **Install PyMeshLab:**  
   With the environment activated, install PyMeshLab:
   ```bash
   pip3 install pymeshlab

10. **Connect MATLAB to Python:**  
    In MATLAB, configure the Python executable for your Anaconda environment (e.g., an environment named "matlab_env") by running:
    
    ```matlab
    pyenv('Version', 'C:\path\to\anaconda3\envs\matlab_env\python.exe')
    ```
    
    Replace `C:\path\to\anaconda3\envs\matlab_env\python.exe` with the full path to your Python executable for the desired environment.


11. **Increase Java Heap Memory in MATLAB:**  
    Navigate to **Home → Preferences → General → Java Heap Memory** in MATLAB and increase the allocated memory if needed.

---

## 4. Configuration and Execution

- **Running the Framework:**  
  - Locate the `matlab` folder within `Artisynth_JawModel`.
  - To run the optimization for a one-segment case, execute `MainOneSegment.m`. The defect type can be chosen within the code (options: Body       (B) or Symphysis (S)).
  - To run the optimization for a two-segment case (e.g., Ramus and Body (RB)), execute `MainTwoSegment.m`.
  - To test the three-stage workflow for a single iteration, you can run `BDefectManual.m`, `SDefectManual.m`, or `RBDefectManual.m`.

---

## 5. Ethics and Data Access

 **Important:**  Due to ethical considerations, the mesh file required for certain processing tasks has not been included in this repository. Researchers or practitioners needing access to this file are requested to contact the authors directly. Additionally, some parameter values in this repository may differ from those reported in the paper due to ongoing experimental adjustments and fine-tuning during development. This repository is actively under reconstruction; your feedback is greatly appreciated.

---

## 6. License

This project is licensed under the GNU General Public License v3. See the [LICENSE](LICENSE) file for details.  Please do not redistribute the current version, as it is intended for review purposes only.

---
