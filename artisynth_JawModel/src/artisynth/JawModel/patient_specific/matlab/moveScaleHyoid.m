import maspack.matrix.*

% Load the JSON file with the extracted transformation
jsonFile = 'translation_scaling.json';  % Path to the JSON file
fid = fopen(jsonFile, 'r');
raw = fread(fid, inf);  % Read the file contents
str = char(raw');  % Convert to character array
fclose(fid);

% Parse the JSON data using jsondecode
jsonData = jsondecode(str);

% Extract translation and scaling
translation = [jsonData.translation(1), jsonData.translation(2), jsonData.translation(3)];
scaling = jsonData.scaling;  % Uniform scaling factor

% Path to muscleInfo.txt file (one folder back, then geometry folder)
muscleInfoFile = '../geometry/muscleInfo.txt';
fid = fopen(muscleInfoFile, 'r');
if fid == -1
    error('Could not open muscleInfo.txt file at %s', muscleInfoFile);
end

% Read muscleInfo.txt line by line
lines = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);
lines = lines{1};

% Step 1: Scale and Translate the Hyoid Rigid Body
hyoidPath = 'models/jawmodel/rigidBodies/hyoid';
hyoidBody = ah1.find(hyoidPath);
if isempty(hyoidBody)
    error('Hyoid bone not found in the model.');
end

% Get the old position of the hyoid
oldHyoidPos = hyoidBody.getPosition();  % Retrieve the current position as Point3d

% Compute the new position of the hyoid by adding the translation
newHyoidPos = Point3d();
newHyoidPos.set(oldHyoidPos);           % Copy the old position
newHyoidPos.add(Point3d(translation));  % Add the translation vector

% Update the hyoid's position
hyoidBody.setPosition(newHyoidPos);  % Update the reference position of the rigid body

% Apply scaling to the surface mesh of the hyoid
hyoidBody.scaleSurfaceMesh(scaling, scaling, scaling);  % Apply uniform scaling

% Step 2: Scale and Translate Insertion and Origin Points
for i = 1:length(lines)
    line = strtrim(lines{i});
    
    % Skip comments or empty lines
    if isempty(line) || startsWith(line, '#')
        continue;
    end
    
    % Parse the line into components
    parts = strsplit(line);
    if length(parts) < 3
        warning('Skipping malformed line: %s', line);
        continue;
    end
    
    muscleName = parts{1};         % First component: muscle name
    originObject = lower(parts{2});  % Second component: origin object
    insertionObject = lower(parts{3});  % Third component: insertion object
    
    % Check if the origin or insertion is associated with the hyoid
    if strcmp(originObject, 'hyoid') || strcmp(insertionObject, 'hyoid')
        % Build the paths for the markers
        insertionPath = sprintf('models/jawmodel/frameMarkers/%s_insertion', muscleName);
        originPath = sprintf('models/jawmodel/frameMarkers/%s_origin', muscleName);
        
        % Update the insertion marker if the insertion object is hyoid
        if strcmp(insertionObject, 'hyoid')
            insertionMarker = ah1.find(insertionPath);
            if ~isempty(insertionMarker)
                % Get the old position and location
                oldInsertionPos = insertionMarker.getPosition();  % Point3d object
                currentInsertionLoc = Point3d();
                insertionMarker.getLocation(currentInsertionLoc);  % Current location
                
                % Apply scaling
                scaledInsertionPos = Point3d(oldInsertionPos);  % Copy the old position
                scaledInsertionPos.scale(scaling);  % Apply uniform scaling

                % Apply translation
                scaledInsertionPos.add(Point3d(translation));  % Add translation vector
                
                % Update the marker's position
                insertionMarker.setPosition(scaledInsertionPos);

                % Update the marker's location
                insertionDifference = Point3d();
                insertionDifference.sub(scaledInsertionPos, oldInsertionPos);  % Difference = newPos - oldPos
                updatedInsertionLoc = Point3d();
                updatedInsertionLoc.add(currentInsertionLoc, insertionDifference);  % updatedLoc = currentLoc + difference
                insertionMarker.setLocation(updatedInsertionLoc);
            end
        end
        
        % Update the origin marker if the origin object is hyoid
        if strcmp(originObject, 'hyoid')
            originMarker = ah1.find(originPath);
            if ~isempty(originMarker)
                % Get the old position and location
                oldOriginPos = originMarker.getPosition();  % Point3d object
                currentOriginLoc = Point3d();
                originMarker.getLocation(currentOriginLoc);  % Current location
                
                % Apply scaling
                scaledOriginPos = Point3d(oldOriginPos);  % Copy the old position
                scaledOriginPos.scale(scaling);  % Apply uniform scaling

                % Apply translation
                scaledOriginPos.add(Point3d(translation));  % Add translation vector
                
                % Update the marker's position
                originMarker.setPosition(scaledOriginPos);

                % Update the marker's location
                originDifference = Point3d();
                originDifference.sub(scaledOriginPos, oldOriginPos);  % Difference = newPos - oldPos
                updatedOriginLoc = Point3d();
                updatedOriginLoc.add(currentOriginLoc, originDifference);  % updatedLoc = currentLoc + difference
                originMarker.setLocation(updatedOriginLoc);
            end
        end
    end
end

% Print confirmation
disp('Scaling and translation applied to the hyoid bone and associated markers.');