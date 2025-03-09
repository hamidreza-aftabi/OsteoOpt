function costFunction = calculateSafetyFactorsCost(leftMatrix, rightMatrix)

    % Find the minimum safety factor from the left matrix
    minLeft = min(leftMatrix(:,2));  % Find the minimum value in the entire left matrix
    
    % Find the minimum safety factor from the right matrix
    minRight = min(rightMatrix(:,2));  % Find the minimum value in the entire right matrix

    % Define the cost function parameters
    leftWeight = .5;  % Weight for the left side
    rightWeight = .5;  % Weight for the right side
    safetyFactorThreshold = 1;  % Example threshold for safety factor

    % Calculate the cost function based on the safety factors
    if minLeft < safetyFactorThreshold
        leftCost = leftWeight * abs((safetyFactorThreshold - minLeft))^2;
    else
        leftCost = 0;
    end

    if minRight < safetyFactorThreshold
        rightCost = rightWeight * abs((safetyFactorThreshold - minRight))^2;
    else
        rightCost = 0;
    end

    % Combine left and right costs into a total cost function
    costFunction = leftCost + rightCost;

    % Display the results
    fprintf('Minimum Safety Factor (Left): %.4f\n', minLeft);
    fprintf('Minimum Safety Factor (Right): %.4f\n', minRight);
    fprintf('Total Safety Cost Function: %.4f\n', costFunction);

end
