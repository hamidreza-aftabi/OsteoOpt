%% Import necessary libraries
import maspack.matrix.*;
import maspack.geometry.*;

%% Load the closest vertex data from the previous code
% The file is assumed to have a header with variable names:
% MuscleName, ClosestInsertionX, ClosestInsertionY, ClosestInsertionZ,
% ClosestOriginX, ClosestOriginY, ClosestOriginZ
closestData = readtable('closest_vertex.txt', 'Delimiter', ' ', 'ReadVariableNames', true);

%% Loop through each muscle and update marker positions and locations
for i = 1:height(closestData)
    % Get the muscle name
    muscleName = closestData.MuscleName{i};
    
    % Create new Point3d objects for the updated marker positions
    newInsertion = Point3d( ...
        closestData.ClosestInsertionX(i), ...
        closestData.ClosestInsertionY(i), ...
        closestData.ClosestInsertionZ(i));
    
    newOrigin = Point3d( ...
        closestData.ClosestOriginX(i), ...
        closestData.ClosestOriginY(i), ...
        closestData.ClosestOriginZ(i));
    
    % --- Update the insertion marker ---
    insertionPath = sprintf('models/jawmodel/frameMarkers/%s_insertion', muscleName);
    insertionMarker = ah1.find(insertionPath);
    if ~isempty(insertionMarker)
        % Get the old insertion position
        oldInsertionPos = Point3d();
        insertionMarker.getPosition(oldInsertionPos);
        
        % Set the new position and update the reference position
        insertionMarker.setPosition(newInsertion);
        insertionMarker.setRefPos(newInsertion);
        
        % Get the current location of the marker
        currentInsertionLoc = Point3d();
        insertionMarker.getLocation(currentInsertionLoc);
        
        % Compute the difference between new and old positions
        insertionDifference = Point3d();
        insertionDifference.sub(newInsertion, oldInsertionPos);  % difference = newPos - oldPos
        
        % Update the marker's location by adding the difference
        updatedInsertionLoc = Point3d();
        updatedInsertionLoc.add(currentInsertionLoc, insertionDifference);
        insertionMarker.setLocation(updatedInsertionLoc);
    end
    
    % --- Update the origin marker ---
    originPath = sprintf('models/jawmodel/frameMarkers/%s_origin', muscleName);
    originMarker = ah1.find(originPath);
    if ~isempty(originMarker)
        % Get the old origin position
        oldOriginPos = Point3d();
        originMarker.getPosition(oldOriginPos);
        
        % Set the new position and update the reference position
        originMarker.setPosition(newOrigin);
        originMarker.setRefPos(newOrigin);
        
        % Get the current location of the marker
        currentOriginLoc = Point3d();
        originMarker.getLocation(currentOriginLoc);
        
        % Compute the difference between new and old positions
        originDifference = Point3d();
        originDifference.sub(newOrigin, oldOriginPos);  % difference = newPos - oldPos
        
        % Update the marker's location by adding the difference
        updatedOriginLoc = Point3d();
        updatedOriginLoc.add(currentOriginLoc, originDifference);
        originMarker.setLocation(updatedOriginLoc);
    end
end

disp('Marker positions and locations updated based on closest vertex output.');
