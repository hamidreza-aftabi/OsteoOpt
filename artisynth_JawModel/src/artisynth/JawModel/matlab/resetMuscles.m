function resetMuscles()

muscleList = fullfile('..', 'geometry', 'muscleList.txt');
muscleInfo = fullfile('..', 'geometry', 'muscleInfo.txt');
closerMuscleList = fullfile('..', 'geometry', 'closerMuscleList.txt');



    % Define the list of muscle names
muscleNames = {
    'rat', 'lat', 'rmt', 'lmt', 'rpt', 'lpt', 'rsm', 'lsm', 'rdm', 'ldm', ...
    'rmp', 'lmp', 'rsp', 'lsp', 'rip', 'lip', 'rad', 'lad', 'ram', 'lam', ...
    'rpm', 'lpm', 'rgh', 'lgh'
};

% Loop through each muscle name and apply the toggleComment function
for i = 1:length(muscleNames)
    muscleName = muscleNames{i};
    toggleComment(muscleList, muscleName, 'remove');
    toggleComment(muscleInfo, muscleName, 'remove');
    toggleComment(closerMuscleList, muscleName, 'remove');
end