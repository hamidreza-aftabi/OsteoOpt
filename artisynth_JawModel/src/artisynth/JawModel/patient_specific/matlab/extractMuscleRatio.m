% Define the file path to the muscle list.
muscleListFile = fullfile('..', 'geometry', 'muscleList.txt');

% Open the file for reading.
fid = fopen(muscleListFile, 'r');
if fid == -1
    error('Could not open file: %s', muscleListFile);
end

% Initialize a cell array to hold valid muscle names.
muscleNames = {};

% Read each line of the file.
while ~feof(fid)
    thisLine = fgetl(fid);
    if isempty(thisLine)
        continue;  % Skip empty lines.
    end
    
    % Trim whitespace.
    thisLine = strtrim(thisLine);
    
    % Skip the line if it starts with a '#' character.
    if startsWith(thisLine, '#')
        continue;
    end
    
    % Add the valid muscle name to the list.
    muscleNames{end+1} = thisLine; %#ok<SAGROW>
end
fclose(fid);

% Prepare a cell array to store the muscle name and computed ratio.
results = {};

% Loop over each muscle name.
for i = 1:numel(muscleNames)
    muscleName = muscleNames{i};
    
    % Build the Artisynth path for the muscle.
    % For a muscle name like 'rsm', the path becomes:
    % "models/jawmodel/axialSprings/rsm"
    musclePath = fullfile('models', 'jawmodel', 'axialSprings', muscleName);
    musclePath = strrep(musclePath, '\', '/');
    
    % Find the muscle in Artisynth.
    muscleObj = ah1.find(musclePath);
    
    if isempty(muscleObj)
        fprintf('Muscle "%s" not found at path "%s".\n', muscleName, musclePath);
        continue;
    end
    
    % Get the material associated with the muscle.
    materialObj = muscleObj.getMaterial();
    
    % Retrieve the max and optimal lengths.
    maxLength = materialObj.getMaxLength();
    optLength = materialObj.getOptLength();
    
    % Check for a potential division by zero.
    if optLength == 0
        fprintf('Optimal length for muscle "%s" is zero; cannot compute ratio.\n', muscleName);
        continue;
    end
    
    % Compute the ratio.
    ratio = maxLength / optLength;
    
    % Save the result (muscle name and ratio) in the results cell array.
    results{end+1, 1} = muscleName; %#ok<SAGROW>
    results{end, 2} = ratio;
    
    fprintf('Muscle: %s | Max/Opt Ratio: %f\n', muscleName, ratio);
end

% Define an output file name in the current working directory.
outputFilename = 'muscleList_ratios.txt';

% Open the file for writing.
fidOut = fopen(outputFilename, 'w');
if fidOut == -1
    error('Could not open output file: %s', outputFilename);
end

% Write each muscle and its ratio to the file.
for i = 1:size(results, 1)
    fprintf(fidOut, '%s %f\n', results{i, 1}, results{i, 2});
end
fclose(fidOut);

fprintf('Results saved to %s\n', outputFilename);
