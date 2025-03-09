function results = helperFunctionTwo(results, currentIteration)


    % Clear unnecessary variables to manage memory
    clearvars -except results currentIteration;

    defectType = 'RB'; 
    trial = 1;

    resultsFile = ['Final_Result_' defectType '_Defect_Trial_' num2str(trial) '.mat'];

    % Define the range of variables and initial points based on defectType
    
    zOffsetRange = [-5, 5];
    rdpOffsetRange = [-7, 7]; 
    leftRollRange = [-25, 25];
    leftPitchRange = [-25, 25];
    rightRollRange = [-15, 15];
    rightPitchRange = [-15, 15];
    initialPoints = [-5, 0, 20, 20, 0, 0];  % Initial point 

    % Use Latin Hypercube Sampling to generate 19 new sample points (20 total with initial point)
    numSamples = 20;  % Number of additional samples to generate (19 + 1 initial point = 20)
    numVariables = 6; % Number of variables to sample

    % Define the variable ranges
    varRanges = [zOffsetRange; rdpOffsetRange; leftRollRange; leftPitchRange; rightRollRange; rightPitchRange];

     
    % Create Sobol sequence generator
    sobolSeq = sobolset(numVariables, 'Skip', 1000, 'Leap', 100); % Skipping initial points for better quality
    sobolSamples = net(sobolSeq, numSamples); % Generate Sobol sequence samples in the range [0,1]
    
    % Scale the Sobol samples to the actual variable ranges
    scaledSamples = bsxfun(@plus, varRanges(:, 1)', bsxfun(@times, sobolSamples, diff(varRanges, [], 2)'));
    

    % Convert the LHS samples to a table for Bayesian optimization
    varsNames = {'zOffset', 'rdpOffset' 'leftRoll', 'leftPitch', 'rightRoll', 'rightPitch'};
    additionalTable = array2table(scaledSamples, 'VariableNames', varsNames);

    % Convert the initial points to a table
    initialTablePoint = array2table(initialPoints, 'VariableNames', varsNames);

    % Combine the initial point with the LHS-generated points
    initialTable = [initialTablePoint; additionalTable];  % Concatenating tables

    % Debugging output to confirm function execution
    disp(['Helper function started with defect type ' defectType ' and LHS sampling with 20 total points.']);

    % Define the optimizable variables
    zOffsetVar = optimizableVariable('zOffset', zOffsetRange);
    rdpOffsetVar = optimizableVariable ('rdpOffset', rdpOffsetRange);
    leftRollVar = optimizableVariable('leftRoll', leftRollRange);
    leftPitchVar = optimizableVariable('leftPitch', leftPitchRange);
    rightRollVar = optimizableVariable('rightRoll', rightRollRange);
    rightPitchVar = optimizableVariable('rightPitch', rightPitchRange);

    % Combine all optimizable variables into an array
    vars = [zOffsetVar, rdpOffsetVar, leftRollVar, leftPitchVar, rightRollVar, rightPitchVar];

    % Check if we are resuming from a previous session
    if exist('results', 'var') && ~isempty(results)
        % Resume Bayesian optimization from previous results
        results = resume(results, ...
            'MaxObjectiveEvaluations', 1);
    else
        % Run Bayesian optimization with the combined initial points
        results = bayesopt(@(params) runArtisynthSimTwo(params), vars, ...
            'Verbose', 1, ...
            'AcquisitionFunctionName', 'expected-improvement-plus', ...
            'ExplorationRatio', 0.6, ...
            'InitialX', initialTable, ...  % Provide the combined initial points
            'MaxObjectiveEvaluations', 1, ... % Initial evaluations
            'IsObjectiveDeterministic', false, ...
            'UseParallel', false);
    end

    % Save the optimization results
    save(resultsFile, 'results', 'currentIteration');

    % Debugging output
    disp('Helper function completed.');

end
