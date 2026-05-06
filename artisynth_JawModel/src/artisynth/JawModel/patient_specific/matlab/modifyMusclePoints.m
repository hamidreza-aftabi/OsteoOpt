% Load transformed muscle data, skipping the header line
import maspack.matrix.*

transformedData = readtable('transformed_muscle_points.txt', 'Delimiter', ' ', 'ReadVariableNames', true);

% Loop through each muscle and update position and location
for i = 1:height(transformedData)
    muscleName = transformedData.MuscleName{i};
    newInsertion = Point3d(transformedData.InsertionX(i), transformedData.InsertionY(i), transformedData.InsertionZ(i));
    newOrigin = Point3d(transformedData.OriginX(i), transformedData.OriginY(i), transformedData.OriginZ(i));
    
    % Update insertion point
    insertionPath = sprintf('models/jawmodel/frameMarkers/%s_insertion', muscleName);
    insertionMarker = ah1.find(insertionPath);
    if ~isempty(insertionMarker)
        % Get the old position
        oldInsertionPos = Point3d();
        insertionMarker.getPosition(oldInsertionPos);
        
        % Set the new position and reference position
        insertionMarker.setPosition(newInsertion);
        insertionMarker.setRefPos(newInsertion);
        
        % Get the current location
        currentInsertionLoc = Point3d();
        insertionMarker.getLocation(currentInsertionLoc);
        
        % Calculate the difference between new and old positions
        insertionDifference = Point3d();
        insertionDifference.sub(newInsertion, oldInsertionPos); % difference = newPos - oldPos
        
        % Update the location
        updatedInsertionLoc = Point3d();
        updatedInsertionLoc.add(currentInsertionLoc, insertionDifference); % updatedLoc = currentLoc + difference
        insertionMarker.setLocation(updatedInsertionLoc);
    end
    
    % Update origin point
    originPath = sprintf('models/jawmodel/frameMarkers/%s_origin', muscleName);
    originMarker = ah1.find(originPath);
    if ~isempty(originMarker)
        % Get the old position
        oldOriginPos = Point3d();
        originMarker.getPosition(oldOriginPos);
        
        % Set the new position and reference position
        originMarker.setPosition(newOrigin);
        originMarker.setRefPos(newOrigin);
        
        % Get the current location
        currentOriginLoc = Point3d();
        originMarker.getLocation(currentOriginLoc);
        
        % Calculate the difference between new and old positions
        originDifference = Point3d();
        originDifference.sub(newOrigin, oldOriginPos); % difference = newPos - oldPos
        
        % Update the location
        updatedOriginLoc = Point3d();
        updatedOriginLoc.add(currentOriginLoc, originDifference); % updatedLoc = currentLoc + difference
        originMarker.setLocation(updatedOriginLoc);
    end
end

disp('Muscle positions and locations updated in ArtiSynth.');
