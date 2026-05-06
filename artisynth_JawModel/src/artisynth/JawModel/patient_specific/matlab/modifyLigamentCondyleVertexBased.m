%% Import necessary libraries
import maspack.matrix.*;
import maspack.geometry.*;

%% Define the markers and target rigid bodies
% Each row: {markerPath, targetRigidBodyName}
pointsToTransform = {
    'models/jawmodel/frameMarkers/CondyleRight', 'jaw_resected';
    'models/jawmodel/frameMarkers/CondyleLeft',  'jaw';
    'models/jawmodel/frameMarkers/tm_sL',         'skull';
    'models/jawmodel/frameMarkers/tm_mL',         'jaw';
    'models/jawmodel/frameMarkers/stm_sL',        'skull';
    'models/jawmodel/frameMarkers/stm_mL',        'jaw';
    'models/jawmodel/frameMarkers/sphm_sL',       'skull';
    'models/jawmodel/frameMarkers/sphm_mL',       'jaw';
    'models/jawmodel/frameMarkers/tm_sR',         'skull';
    'models/jawmodel/frameMarkers/tm_mR',         'jaw_resected';
    'models/jawmodel/frameMarkers/stm_sR',        'skull';
    'models/jawmodel/frameMarkers/stm_mR',        'jaw_resected';
    'models/jawmodel/frameMarkers/sphm_sR',       'skull';
    'models/jawmodel/frameMarkers/sphm_mR',       'jaw_resected';
};

%% Loop through each marker and update its position to the closest vertex
for i = 1:size(pointsToTransform, 1)
    markerPath = pointsToTransform{i,1};
    targetRBName = pointsToTransform{i,2};
    
    % Retrieve the marker object using its path
    marker = ah1.find(markerPath);
    if isempty(marker)
        warning('Marker not found: %s', markerPath);
        continue;
    end
    
    % Build the target rigid body path and retrieve the rigid body
    targetRigidBodyPath = sprintf('models/jawmodel/rigidBodies/%s', targetRBName);
    targetRigidBody = ah1.find(targetRigidBodyPath);
    if isempty(targetRigidBody)
        warning('Rigid body not found: %s', targetRigidBodyPath);
        continue;
    end
    
    % Get the marker's current position (our arbitrary point)
    arbitraryPoint = marker.getPosition();
    
    % Get the Artisynth root so we can call our custom Java method
    root = ah1.root();
    
    % Call the Java method to find the closest vertex on the target rigid body
    closestVertex = root.findClosestVertex(targetRigidBody, arbitraryPoint);
    if isempty(closestVertex)
        warning('No closest vertex found for marker: %s', markerPath);
        continue;
    end

    
    % Update the marker's location:
    currentLoc = Point3d();
    marker.getLocation(currentLoc);
    
    diff = Point3d();
    diff.sub(closestVertex, arbitraryPoint);  % diff = closestVertex - old position
    
    updatedLoc = Point3d();
    updatedLoc.add(currentLoc, diff);
    marker.setLocation(updatedLoc);

    % --- Update the marker ---
    % Move the marker to the closest vertex position without reparenting.
    marker.setPosition(closestVertex);
    marker.setRefPos(closestVertex);
    
    fprintf('Updated marker %s to closest vertex on %s at (%.6f, %.6f, %.6f)\n', ...
        markerPath, targetRBName, closestVertex.x, closestVertex.y, closestVertex.z);
end

%% Optionally force a refresh if needed
% ah1.repaint();

disp('All markers moved to the closest vertex positions on their corresponding rigid bodies.');
