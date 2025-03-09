function removeRBMuscles ()

muscleList = fullfile('..', 'geometry', 'muscleList.txt');
muscleInfo = fullfile('..', 'geometry', 'muscleInfo.txt');
closerMuscleList = fullfile('..', 'geometry', 'closerMuscleList.txt');

muscleNames = {
    'rad', 'rpm', 'rat', 'rmt', 'rpt', 'rsm', 'rdm', 'rmp' ,'ram'
};


% Loop through each muscle name and apply the toggleComment function
for i = 1:length(muscleNames)
    muscleName = muscleNames{i};
    toggleComment(muscleList, muscleName, 'add');
    toggleComment(muscleInfo, muscleName, 'add');
    toggleComment(closerMuscleList, muscleName, 'add');
end