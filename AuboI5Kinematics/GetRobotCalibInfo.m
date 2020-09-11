function [calibBools, numParams, numParamsTotal] = GetRobotCalibInfo()

% Nominal observables given a standard robot calibration 
% calibBools = logical([1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 0 0 0 1 1 1 1 1 1]);

% Observables if we want to calibrate the full robot
% calibBools =   logical([0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 0 0 0 1 1 1 0 0 0]);

% Observables if we want to only calibrate the IMU transform
calibBools = logical([zeros(1, 6*6), ones(1, 3), zeros(1, 3)]);

numParamsTotal = length(calibBools);
numParams = sum(calibBools);

end