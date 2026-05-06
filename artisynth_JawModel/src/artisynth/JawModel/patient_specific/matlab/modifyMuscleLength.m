% updateMuscleLengths.m
%
% This script reads the muscle names and ratio values from
% muscleList_ratios.txt, then for each muscle, it:
%   1. Retrieves its current length using getLength.
%   2. Sets its optimal length (using setOptLenght) to the current length.
%   3. Sets its maximum length (using setMaxLenght) to current length * ratio.
%
% Make sure your MATLAB session is connected to Artisynth and that the
% handle 'ah1' is available.

% Define the filename for the ratios file.
ratiosFilename = 'muscleList_ratios.txt';

% Open the file for reading.
fid = fopen(ratiosFilename, 'r');
if fid == -1
    error('Could not open file: %s', ratiosFilename);
end

% Read the file.
% Assuming each line has: muscleName ratio
data = textscan(fid, '%s %f');
fclose(fid);

% Extract muscle names and their corresponding ratios.
muscleNames = data{1};
ratios = data{2};

% Process each muscle.
for i = 1:length(muscleNames)
    muscleName = muscleNames{i};
    ratio = ratios(i);
    
    % Build the Artisynth path for the muscle.
    % For a muscle like 'rsm', we expect the path to be:
    % "models/jawmodel/axialSprings/rsm"
    musclePath = fullfile('models', 'jawmodel', 'axialSprings', muscleName);
    % Replace backslashes with forward slashes if necessary.
    musclePath = strrep(musclePath, '\', '/');
    
    % Find the muscle in Artisynth using the assumed handle 'ah1'.
    muscleObj = ah1.find(musclePath);
    if isempty(muscleObj)
        fprintf('Muscle "%s" not found at path "%s".\n', muscleName, musclePath);
        continue;
    end
    
    % Get the current length of the muscle using getLength().
    L = muscleObj.getLength();
    
    % Retrieve the material object of the muscle.
    materialObj = muscleObj.getMaterial();
    
    % Set the optimal length of the muscle material to the current length.
    materialObj.setOptLength(L);
    
    % Set the maximum length as current length multiplied by the ratio.
    materialObj.setMaxLength(L * ratio);
    
    fprintf('Updated muscle "%s": optLenght set to %f, maxLenght set to %f.\n', ...
            muscleName, L, L * ratio);
end

fprintf('Muscle length updates complete.\n');
