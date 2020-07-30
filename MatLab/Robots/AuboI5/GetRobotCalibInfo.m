function [calibBools, numParams, numParamsTotal, paramsMeters] = GetRobotCalibInfo()

% Nominal observables given a standard robot calibration 
% calibBools = logical([1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 0 0 0 1 1 1 1 1 1]);

% Observables if we want to calibrate the full robot
% calibBools =   logical([0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 1 0 1 1 1 0 1 0 1 1 1 0 1 0 1 1 1 0 0 0 0 1 1 1 1 1 1]);

% Observables if we want to only calibrate the IMU transform
calibBools = logical([zeros(1, 6*6), ones(1, 6)]);

numParamsTotal = length(calibBools);
numParams = sum(calibBools);
paramsMeters = logical(reshape([ones(3, 7); zeros(3, 7)], 1, 6*7));
end