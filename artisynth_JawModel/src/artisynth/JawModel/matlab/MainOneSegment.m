clc;
clear all;

defectType = 'B';
trial = 1;
resultsFile = ['Sobol_Final_Result_Sensitivity_' defectType '_Defect_Trial_' num2str(trial) '.mat'];

% Number of total iterations
totalIterations = 50;



% Load previous results if they exist
if isfile(resultsFile)
    load(resultsFile, 'results', 'currentIteration');
else
    results = [];
    currentIteration = 1;
    rng('shuffle');
end

% Perform optimization in iterations
while currentIteration <= totalIterations
    disp(['Starting iteration: ' num2str(currentIteration)]);
  
    try
        % Run the helper function
        results = helperFunctionV4WithSobolSeq(results, currentIteration);
        
        % Save results and current iteration
        currentIteration = currentIteration + 1;
        save(resultsFile, 'results', 'currentIteration');
        
        % Check if we need to continue and restart MATLAB
        if currentIteration <= totalIterations
            disp('Restarting MATLAB to manage memory...');
            % Save workspace and restart MATLAB
            save(resultsFile, 'results', 'currentIteration', 'totalIterations');
            % Use system command to restart MATLAB
            matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
            matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename('fullpath'));
            system(matlabCommand);
            exit; % Close the current MATLAB session
        end
    catch ME
        % Handle specific Java exception
        if strcmp(ME.identifier, 'MATLAB:Java:GenericException')
            javaEx = ME.ExceptionObject;
            if isa(javaEx, 'maspack.matrix.NumericalException') 
                 disp('*************NumericalException:  Inverted elements************');
                 pause(1)
                 matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
                 matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename('fullpath'));
                 system(matlabCommand);
                 exit; % Close the current MATLAB session
                
            elseif  isa(javaEx, 'maspack.util.InternalErrorException')
                 disp('**************Cut Error**************');
                 pause(1)
                 matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
                 matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename('fullpath'));
                 system(matlabCommand);
                 exit; % Close the current MATLAB session
            elseif isa(javaEx, 'maspack.matrix.NumericalException: findNextDonorPoint') 
                 disp('**************Next Point Error**************');
                 pause(1)
                 matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
                 matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename('fullpath'));
                 system(matlabCommand);
                 exit; % Close the current MATLAB session
            else
                disp(['Java exception occurred: ' ME.message]);
            end

        else
            disp(['Error occurred: ' ME.message]);
            disp('MATLAB Error...');
            pause(1)
            % Save workspace and restart MATLAB
            save(resultsFile, 'results', 'currentIteration', 'totalIterations');
            % Use system command to restart MATLAB
            matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
            matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename('fullpath'));
            system(matlabCommand);
            exit; % Close the current MATLAB session
        end
        
    end
end
