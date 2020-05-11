function ChangeRobot(robotName)

% Hard code robot dir
robotsDir = fullfile(pwd, '..', 'Robots');

% Remove all robot files from path
addpath(genpath(robotsDir));
rmpath(genpath(robotsDir));

% Add the robot files of interest to the path after ensuring existance
robotDir = fullfile(robotsDir, robotName);

if exist(robotDir, 'dir')
    addpath(genpath(robotDir))
else
    error('Robot path %s does not exist', robotDir);
end

end