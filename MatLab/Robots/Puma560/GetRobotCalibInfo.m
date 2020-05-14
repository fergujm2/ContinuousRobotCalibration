function [calibBools, numParams, numParamsTotal, paramsMeters] = GetRobotCalibInfo()

% calibBools = logical([1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   0   0   0]);
% calibBools   = logical([0   0   0   0   0   0   0   0   0   0   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   1   1   1]);
calibBools     = logical([0   0   0   0   0   0   0   0   0   0   0   1   0   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   1   1   1]);
% calibBools     = logical([0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1]);

numParamsTotal = length(calibBools);
numParams = sum(calibBools);
paramsMeters = logical(reshape([ones(3, 7); zeros(3, 7)], 1, 42));
end