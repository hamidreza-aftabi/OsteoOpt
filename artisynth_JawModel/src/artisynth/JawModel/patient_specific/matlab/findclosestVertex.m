%% Import necessary libraries
import maspack.matrix.*;
import maspack.geometry.*;

%% Define file paths
% Change these paths as needed
muscleInfoFile = '../geometry/muscleInfo.txt';  % Path to muscleInfo.txt
outputFile     = 'closest_vertex.txt';          % Output file for results

%% Read muscleInfo.txt line by line
fid = fopen(muscleInfoFile, 'r');
if fid == -1
    error('Could not open muscleInfo.txt file at %s', muscleInfoFile);
end
lines = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);
lines = lines{1};

%% Open output file for writing results
fid_out = fopen(outputFile, 'w');
fprintf(fid_out, 'MuscleName ClosestInsertionX ClosestInsertionY ClosestInsertionZ ClosestOriginX ClosestOriginY ClosestOriginZ\n');

%% Process each muscle entry
for i = 1:length(lines)
    line = strtrim(lines{i});
    
    % Skip empty lines or comment lines (e.g., starting with '#')
    if isempty(line) || startsWith(line, '#')
        continue;
    end

    % Parse the line into parts (expecting at least 3 parts)
    parts = strsplit(line);
    if length(parts) < 3
        warning('Skipping malformed line: %s', line);
        continue;
    end
    
    muscleName     = parts{1};         % Muscle name
    originObject   = lower(parts{2});    % Origin object name
    insertionObject = lower(parts{3});   % Insertion object name
    
    % Build marker paths (adjust these paths to match your model)
    insertionMarkerPath = sprintf('models/jawmodel/frameMarkers/%s_insertion', muscleName);
    originMarkerPath    = sprintf('models/jawmodel/frameMarkers/%s_origin', muscleName);
    
    % Retrieve markers using the Artisynth handle (ah1)
    insertionMarker = ah1.find(insertionMarkerPath);
    originMarker    = ah1.find(originMarkerPath);
    
    if isempty(insertionMarker) || isempty(originMarker)
        warning('Skipping %s: one or both markers not found.', muscleName);
        continue;
    end
    
    % Get the marker positions (returned as a Point3d)
    insertionPoint = insertionMarker.getPosition();
    originPoint    = originMarker.getPosition();
    
    % Build the rigid body paths (adjust these paths to match your model)
    insertionRigidBodyPath = sprintf('models/jawmodel/rigidBodies/%s', insertionObject);
    originRigidBodyPath    = sprintf('models/jawmodel/rigidBodies/%s', originObject);
    
    % Retrieve the rigid bodies
    insertionRigidBody = ah1.find(insertionRigidBodyPath);
    originRigidBody    = ah1.find(originRigidBodyPath);
    
    if isempty(insertionRigidBody) || isempty(originRigidBody)
        warning('Skipping %s: one or both rigid bodies not found.', muscleName);
        continue;
    end
    
    %% Call the Java method via the Artisynth root to find closest vertices
    root = ah1.root();  % Get the root component (which implements findClosestVertex)
    
    % Call the method for the insertion marker:
    closestInsertionVertex = root.findClosestVertex(insertionRigidBody, insertionPoint);
    
    % Call the method for the origin marker:
    closestOriginVertex = root.findClosestVertex(originRigidBody, originPoint);
    
    % Check if either call returned null (or empty)
    if isempty(closestInsertionVertex) || isempty(closestOriginVertex)
        warning('Skipping %s: closest vertex not found for one or both markers.', muscleName);
        continue;
    end
    
    %% Write the results to the output file
    fprintf(fid_out, '%s %.6f %.6f %.6f %.6f %.6f %.6f\n', muscleName, ...
        closestInsertionVertex.x, closestInsertionVertex.y, closestInsertionVertex.z, ...
        closestOriginVertex.x, closestOriginVertex.y, closestOriginVertex.z);
end

%% Close the output file and display a confirmation message
fclose(fid_out);
disp('Closest vertices saved to closest_vertex.txt');
