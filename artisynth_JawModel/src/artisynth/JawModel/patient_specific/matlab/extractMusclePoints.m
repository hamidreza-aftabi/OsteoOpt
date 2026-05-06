% Define muscle names
muscleNames = {'rat', 'lat', 'rmt', 'lmt', 'rpt', 'lpt', 'rsm', 'lsm', 'rdm', 'ldm', 'rmp', 'lmp', 'rsp', 'lsp', 'rip', 'lip', 'lad', 'lam', 'rpm', 'lpm', 'rgh', 'lgh'};

% Open a file to save muscle data
fileID = fopen('muscle_points.txt', 'w');

% Write header to the file
fprintf(fileID, 'MuscleName InsertionX InsertionY InsertionZ OriginX OriginY OriginZ\n');

% Loop through each muscle
for i = 1:length(muscleNames)
    muscleName = muscleNames{i};
    
    % Get the insertion and origin points using ArtiSynth and mmat
    insertionPath = sprintf('models/jawmodel/frameMarkers/%s_insertion', muscleName);
    originPath = sprintf('models/jawmodel/frameMarkers/%s_origin', muscleName);
    
    insertionMarker = ah1.find(insertionPath); % Find the insertion marker
    originMarker = ah1.find(originPath);       % Find the origin marker
    
    if ~isempty(insertionMarker) && ~isempty(originMarker)
        % Use mmat to handle the position
        insertionPoint = mmat(insertionMarker.getPosition()); % Use mmat here
        originPoint = mmat(originMarker.getPosition());       % Use mmat here
        
        % Save the data in text format
        fprintf(fileID, '%s %.6f %.6f %.6f %.6f %.6f %.6f\n', ...
                muscleName, ...
                insertionPoint(1), insertionPoint(2), insertionPoint(3), ...
                originPoint(1), originPoint(2), originPoint(3));
    else
        warning('Could not find markers for muscle: %s', muscleName);
    end
end

% Close the file
fclose(fileID);

disp('Muscle data saved to muscle_points.txt');
