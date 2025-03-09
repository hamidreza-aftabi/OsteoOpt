function [new_axis, new_angle] = rotate_axis_angle_around_local_x(axis, angle_deg, alpha_deg)
    % Normalize the axis
    axis = axis / norm(axis);
    
    % Convert the initial angle to radians
    theta = deg2rad(angle_deg);
    
    % Construct the skew-symmetric matrix K
    nx = axis(1);
    ny = axis(2);
    nz = axis(3);
    K = [0, -nz, ny;
         nz, 0, -nx;
         -ny, nx, 0];
    
    % Calculate sin(theta) and cos(theta)
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    
    % Calculate the rotation matrix R using Rodrigues' formula
    I = eye(3);
    R = I + sin_theta * K + (1 - cos_theta) * K^2;
    
    % Additional rotation around the local x-axis by alpha degrees
    alpha_rad = deg2rad(alpha_deg);
    R_x_local = [1, 0, 0;
                 0, cos(alpha_rad), -sin(alpha_rad);
                 0, sin(alpha_rad), cos(alpha_rad)];
    
    % Combine the rotations
    R_combined = R * R_x_local;
    
    % Convert the resulting rotation matrix back to axis-angle representation
    % Calculate the angle of rotation
    trace_R = trace(R_combined);
    new_theta = acos((trace_R - 1) / 2);
    new_angle = rad2deg(new_theta);
    
    % Calculate the rotation axis
    sin_new_theta = sin(new_theta);
    if sin_new_theta ~= 0
        nx = (R_combined(3, 2) - R_combined(2, 3)) / (2 * sin_new_theta);
        ny = (R_combined(1, 3) - R_combined(3, 1)) / (2 * sin_new_theta);
        nz = (R_combined(2, 1) - R_combined(1, 2)) / (2 * sin_new_theta);
        new_axis = [nx, ny, nz];
    else
        % Handle the case where sin(new_theta) is close to zero
        % This occurs when the rotation angle is 0 or 180 degrees
        new_axis = [1, 0, 0]; % Default to x-axis if angle is 0 or 180 degrees
    end
end


