% updateLigamentRestLength.m
%
% This script updates the rest length of two ligament types:
%   - "stm": for which the new rest length = current length + 1.5
%   - "sphm": for which the new rest length = current length + 5.5
%
% The script assumes that each ligament can be found under the path:
%   models/jawmodel/axialSprings/<ligamentName>
% where the left ligament is named with a "_L" suffix and the right with "_R".
%
% Example: left stm ligament is at "models/jawmodel/axialSprings/stm_L".
% It gets the current length using getLength() and sets the new rest length
% using setRestLenght(newLength).

% Define the ligament types and the corresponding offsets.
ligamentTypes = {'stm', 'sphm'};    % ligament base names
offsets = [1.5, 5.5];               % offsets: for stm add 1.5, for sphm add 5.5

% Define the sides (left and right).
sides = {'L', 'R'};

% Loop over each ligament type.
for i = 1:length(ligamentTypes)
    baseType = ligamentTypes{i};
    offset = offsets(i);
    
    % Process each side.
    for j = 1:length(sides)
        side = sides{j};
        % Construct the full ligament name. For example: "stm_L" or "sphm_R".
        ligamentName = [baseType, '_', side];
        
        % Construct the Artisynth path.
        % Expected path: "models/jawmodel/axialSprings/<ligamentName>"
        ligamentPath = fullfile('models', 'jawmodel', 'axialSprings', ligamentName);
        % Replace backslashes with forward slashes if necessary.
        ligamentPath = strrep(ligamentPath, '\', '/');
        
        % Find the ligament object using the Artisynth handle (ah1).
        ligamentObj = ah1.find(ligamentPath);
        if isempty(ligamentObj)
            fprintf('Ligament "%s" not found at path "%s".\n', ligamentName, ligamentPath);
            continue;
        end
        
        % Get the current length of the ligament.
        currentLength = ligamentObj.getLength();
        
        % Compute the new rest length.
        newRestLength = currentLength + offset;
        
        % Set the new rest length.
        ligamentObj.setRestLength(newRestLength);
        
        fprintf('Updated ligament "%s": current length = %f, new rest length = %f (offset = %f).\n', ...
            ligamentName, currentLength, newRestLength, offset);
    end
end

fprintf('Ligament rest length updates complete.\n');
