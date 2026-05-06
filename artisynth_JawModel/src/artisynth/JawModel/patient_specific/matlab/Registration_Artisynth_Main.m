artisynthHome = getenv('ARTISYNTH_HOME');
if isempty(artisynthHome)
    error('ARTISYNTH_HOME must point to the local artisynth_core checkout.');
end

addpath(fullfile(artisynthHome, 'matlab'));
setArtisynthClasspath(artisynthHome);

import maspack.matrix.*

GeometryDir = fullfile('..','geometry');
modelClassName = 'artisynth.istar.TMJModel.Case1.JawFemDemoOptimize';


%%%%%%%%%%%%%%%%%%%%%
pyrunfile ('findRigidRegistration.py')
pyrunfile ('applyRigidRegistration.py')

pyrunfile('deformableRegistration_mand.py')
pyrunfile('deformableRegistration_skull.py')

pyrunfile('deformableRegistration_disc.py')

requiredTmjPipelines = {
    'registration_pipeline_condyle_right.json'
    'registration_pipeline_fossa_right.json'
};

if all(cellfun(@(f) isfile(fullfile(pwd, f)), requiredTmjPipelines))
    pyrunfile('applyDualDeformRegistration_disc.py')
else
    warning(['Skipping dual disc deformation because one or more TMJ ' ...
        'registration pipeline JSON files are missing. Generate the ' ...
        'condyle and fossa registration_pipeline_*.json files first.']);
end

% Optional capsule adaptation helper:
% pyrunfile('applyDualDeformRegistration_capsule.py')
%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%
pyrunfile ('simpMesh.py')
pyrunfile ('substractMesh.py')
pyrunfile('remeshMesh.py')
%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%
resetMuscles();
removeRBMusclesPatient1;
%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%
ah1 = artisynth('-model', modelClassName);
%invisibleComponents
%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%
visibleComponents1
visibleComponents2
%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%
removeLigamentsPatient1;

extractMusclePoints
pyrunfile('applyRegistrationToPoints.py')
modifyMusclePoints

pyrunfile('extractSimilarityWeight.py')
moveScaleHyoid;

modifyLigamentCondyle
%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%
findclosestVertex
modifyMusclePointsVertexBased
modifyLigamentCondyleVertexBased
%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%
extractMuscleRatio
modifyMuscleLength
modifyPCSA
modifyLigamentLength
%%%%%%%%%%%%%%%%%%%%%

