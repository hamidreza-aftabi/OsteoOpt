% updateMuscleForce.m
% This script reads the PCSA.txt file that contains muscle base names and 
% their maxForce values. For each base name (e.g., 'sm'), it updates both 
% the left (e.g., 'lsm') and right (e.g., 'rsm') muscles in Artisynth by:

% Define the filename for the PCSA file.
pcsaFilename = 'PCSA.txt';

% Open the file for reading.
fid = fopen(pcsaFilename, 'r');
if fid == -1
    error('Could not open file: %s', pcsaFilename);
end

% Read the file. Each line should contain a muscle base name and a force value.
data = textscan(fid, '%s %f');
fclose(fid);

% Extract muscle base names and force values.
muscleBases = data{1};
forces = data{2};

% Loop over each muscle base.
for i = 1:length(muscleBases)
    baseName = muscleBases{i};
    maxForceValue = forces(i);
    
    % Create left and right muscle names.
    % For example, for baseName 'sm', we generate 'lsm' and 'rsm'.
    sidePrefixes = {'l', 'r'};
    for j = 1:length(sidePrefixes)
        muscleName = [sidePrefixes{j} baseName];
        
        % Construct the muscle path.
        % Expected path: "models/jawmodel/axialSprings/<muscleName>"
        musclePath = fullfile('models', 'jawmodel', 'axialSprings', muscleName);
        % Convert backslashes to forward slashes if necessary.
        musclePath = strrep(musclePath, '\', '/');
        
        % Find the muscle object in Artisynth.
        muscleObj = ah1.find(musclePath);
        if isempty(muscleObj)
            fprintf('Muscle "%s" not found at path "%s".\n', muscleName, musclePath);
            continue;
        end
        
        % Get the material object from the muscle.
        materialObj = muscleObj.getMaterial();
        
        % Set the maxForce using the material's method.
        materialObj.setMaxForce(maxForceValue);
        fprintf('Updated muscle "%s": maxForce set to %f.\n', muscleName, maxForceValue);
    end
end

fprintf('Muscle force updates complete.\n');
