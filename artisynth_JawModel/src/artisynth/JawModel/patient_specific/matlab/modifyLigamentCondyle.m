% Import necessary libraries
import maspack.matrix.*;

% Load the JSON files with deformation weights
mandibleJsonFile = 'deformation_weights_mandible.json'; % Path to mandible weights
maxillaJsonFile = 'deformation_weights_maxilla.json';   % Path to maxilla weights

% Load weights
mandibleWeights = load_weights(mandibleJsonFile);
maxillaWeights = load_weights(maxillaJsonFile);

% Define the points to be transformed
pointsToTransform = {
    'models/jawmodel/frameMarkers/CondyleRight', 'mandible';
    'models/jawmodel/frameMarkers/CondyleLeft', 'mandible';
    'models/jawmodel/frameMarkers/tm_sL', 'skull';
    'models/jawmodel/frameMarkers/tm_mL', 'mandible';
    'models/jawmodel/frameMarkers/stm_sL', 'skull';
    'models/jawmodel/frameMarkers/stm_mL', 'mandible';
    'models/jawmodel/frameMarkers/sphm_sL', 'skull';
    'models/jawmodel/frameMarkers/sphm_mL', 'mandible';
    'models/jawmodel/frameMarkers/tm_sR', 'skull';
    'models/jawmodel/frameMarkers/tm_mR', 'mandible';
    'models/jawmodel/frameMarkers/stm_sR', 'skull';
    'models/jawmodel/frameMarkers/stm_mR', 'mandible';
    'models/jawmodel/frameMarkers/sphm_sR', 'skull';
    'models/jawmodel/frameMarkers/sphm_mR', 'mandible';
};

% Process each point
for i = 1:size(pointsToTransform, 1)
    pointPath = pointsToTransform{i, 1};
    reference = pointsToTransform{i, 2};

    % Find the point in the ArtiSynth model
    marker = ah1.find(pointPath);
    if isempty(marker)
        warning('Marker not found: %s', pointPath);
        continue;
    end

    % Get the current position and location
    oldPosition = Point3d();
    marker.getPosition(oldPosition);

    currentLocation = Point3d();
    marker.getLocation(currentLocation);

    % Convert position to array
    oldPositionArray = [oldPosition.x, oldPosition.y, oldPosition.z];

    % Apply transformation based on the reference
    if strcmp(reference, 'mandible')
        newPositionArray = apply_cpd_transform( ...
            oldPositionArray, ...
            mandibleWeights.Y, ...
            mandibleWeights.W, ...
            mandibleWeights.beta ...
        );
    elseif strcmp(reference, 'skull')
        newPositionArray = apply_cpd_transform( ...
            oldPositionArray, ...
            maxillaWeights.Y, ...
            maxillaWeights.W, ...
            maxillaWeights.beta ...
        );
    else
        newPositionArray = oldPositionArray;
    end

    % Convert the new position back to Point3d
    newPosition = Point3d(newPositionArray(1), newPositionArray(2), newPositionArray(3));

    % Update marker position and reference position
    marker.setPosition(newPosition);
    marker.setRefPos(newPosition);

    % Update marker location by the same displacement
    positionDifference = Point3d();
    positionDifference.sub(newPosition, oldPosition);

    updatedLocation = Point3d();
    updatedLocation.add(currentLocation, positionDifference);
    marker.setLocation(updatedLocation);
end

disp('Transformation applied to specified points, and both position and location updated.');


% ------------------------------------------------------------
% Helper: load weights
% ------------------------------------------------------------
function weights = load_weights(jsonFile)
    fid = fopen(jsonFile, 'r');
    if fid == -1
        error('Could not open file: %s', jsonFile);
    end

    raw = fread(fid, inf);
    fclose(fid);

    str = char(raw');
    jsonData = jsondecode(str);

    weights.Y = reshape(jsonData.Y, [], 3);
    weights.W = reshape(jsonData.W, [], 3);
    weights.beta = jsonData.beta;
end


% ------------------------------------------------------------
% Helper: Gaussian kernel
% ------------------------------------------------------------
function G = gaussian_kernel(X, Y, beta)
    % X: Nx3
    % Y: Mx3
    % G: NxM
    N = size(X, 1);
    M = size(Y, 1);
    G = zeros(N, M);

    for i = 1:N
        diff = Y - X(i, :);
        dist2 = sum(diff.^2, 2);
        G(i, :) = exp(-dist2 / (2.0 * beta^2));
    end
end


% ------------------------------------------------------------
% Helper: reconstruct control-point shift
% TY - Y = G(Y,Y) @ W
% ------------------------------------------------------------
function controlShift = reconstruct_control_shift(Y, W, beta)
    Gyy = gaussian_kernel(Y, Y, beta);
    controlShift = Gyy * W;
end


% ------------------------------------------------------------
% Helper: apply normalized difference-based transfer
% shift(point) = (G(point,Y) @ (TY - Y)) / sum(G)
% ------------------------------------------------------------
function transformedPoint = apply_cpd_transform(point, Y, W, beta)
    point = reshape(point, 1, 3);

    % Reconstruct TY - Y from saved CPD params
    controlShift = reconstruct_control_shift(Y, W, beta);

    % Gaussian weights from point to control points
    G = gaussian_kernel(point, Y, beta);   % 1xM
    sumG = sum(G, 2);

    if sumG == 0
        sumG = 1e-12;
    end

    weightedShift = (G * controlShift) / sumG;   % 1x3
    transformedPoint = point + weightedShift;
end