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

missingTmjPipelines = requiredTmjPipelines(~cellfun(@(f) isfile(fullfile(pwd, f)), requiredTmjPipelines));
if ~isempty(missingTmjPipelines)
    error(['TMJ disc and capsule deformation requires these generated ' ...
        'registration pipeline files: %s'], strjoin(missingTmjPipelines, ', '));
end

pyrunfile('applyDualDeformRegistration_disc.py')
pyrunfile('applyDualDeformRegistration_capsule.py')
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

