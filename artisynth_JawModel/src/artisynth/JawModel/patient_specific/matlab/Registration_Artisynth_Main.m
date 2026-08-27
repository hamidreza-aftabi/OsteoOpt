artisynthHome = getenv('ARTISYNTH_HOME');
if isempty(artisynthHome)
    error('ARTISYNTH_HOME must point to the local artisynth_core checkout.');
end

addpath(fullfile(artisynthHome, 'matlab'));
setArtisynthClasspath(artisynthHome);
import maspack.matrix.*

registrationDir = fileparts(mfilename('fullpath'));
slicerExecutable = getenv('SLICER_EXECUTABLE');
if isempty(slicerExecutable) || ~isfile(slicerExecutable)
    error('SLICER_EXECUTABLE must point to Slicer.exe.');
end
scsaCtFile = fullfile(registrationDir, 'inputs', 'CT.nrrd');
scsaLandmarksFile = fullfile(registrationDir, 'inputs', 'SCSA_landmarks.json');
scsaOutputFile = fullfile(registrationDir, 'SCSA.txt');
scsaScript = fullfile(registrationDir, 'generateSCSA.py');
scsaStatusFile = fullfile(registrationDir, '.SCSA.status');
useNeighboringScsPlanes = false;
if ~isfile(scsaCtFile)
    error('SCSA CT file not found: %s', scsaCtFile);
end
if ~isfile(scsaLandmarksFile)
    error('SCSA landmark file not found: %s', scsaLandmarksFile);
end
if isfile(scsaStatusFile)
    delete(scsaStatusFile);
end
scsaArguments = sprintf('"%s" --no-splash --no-main-window --ignore-slicerrc --modules-to-ignore Telemetry,BoneReconstructionPlanner --python-script "%s" -- --ct "%s" --landmarks "%s" --output "%s" --status-file "%s"', slicerExecutable, scsaScript, scsaCtFile, scsaLandmarksFile, scsaOutputFile, scsaStatusFile);
if useNeighboringScsPlanes
    scsaArguments = [scsaArguments ' --neighboring-planes'];
end
if ispc
    scsaCommand = ['start "" /wait ' scsaArguments];
else
    scsaCommand = scsaArguments;
end
[~, scsaLog] = system(scsaCommand);
if ~isfile(scsaStatusFile)
    error('SCSA generation did not complete:\n%s', scsaLog);
end
scsaStatus = fileread(scsaStatusFile);
delete(scsaStatusFile);
if ~startsWith(scsaStatus, 'SUCCESS') || ~isfile(scsaOutputFile)
    error('SCSA generation failed:\n%s', scsaStatus);
end

tmjSides = {'left', 'right'};
tmjRegions = {'condyle', 'fossa'};

pyrunfile('findRigidRegistration.py');
pyrunfile('applyRigidRegistration.py');
pyrunfile('deformableRegistration_mand.py');
pyrunfile('deformableRegistration_skull.py');

for sideIndex = 1:numel(tmjSides)
    setenv('TMJ_SIDE', tmjSides{sideIndex});
    for regionIndex = 1:numel(tmjRegions)
        setenv('TMJ_REGION', tmjRegions{regionIndex});
        pyrunfile('deformableRegistration_disc.py');
    end
end

requiredTmjPipelines = {
    'registration_pipeline_condyle_left.json'
    'registration_pipeline_fossa_left.json'
    'registration_pipeline_condyle_right.json'
    'registration_pipeline_fossa_right.json'
};
missingTmjPipelines = requiredTmjPipelines(~cellfun(@isfile, requiredTmjPipelines));
if ~isempty(missingTmjPipelines)
    error('Missing TMJ registration pipelines: %s', strjoin(missingTmjPipelines, ', '));
end

for sideIndex = 1:numel(tmjSides)
    setenv('TMJ_SIDE', tmjSides{sideIndex});
    pyrunfile('applyDualDeformRegistration_disc.py');
    pyrunfile('applyDualDeformRegistration_capsule.py');
end
setenv('TMJ_SIDE', '');
setenv('TMJ_REGION', '');

pyrunfile('simpMesh.py');
pyrunfile('substractMesh.py');
pyrunfile('remeshMesh.py');

resetMuscles();
removeRBMusclesPatient1;

% For two-segment reconstruction, use: ah1 = artisynth('-model', 'artisynth.JawModel.JawFemDemoOptimizeTwoWithSafety');
ah1 = artisynth('-model', 'artisynth.JawModel.JawFemDemoOptimize');
removeLigamentsPatient1;

extractMusclePoints;
pyrunfile('applyRegistrationToPoints.py');
modifyMusclePoints;
pyrunfile('extractSimilarityWeight.py');
moveScaleHyoid;
modifyLigamentCondyle;

findclosestVertex;
modifyMusclePointsVertexBased;
modifyLigamentCondyleVertexBased;

extractMuscleRatio;
modifyMuscleLength;
calculateFMAX;
modifyFMAX;
modifyLigamentLength;
