function [y, alph, omeg] = ImuMeasurementEquation(theta, t, q, qDot, qDDot)

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
[fitTimeOffset, fitScaleFactors] = GetCalibOptions();

[x, g, tau, s] = UnpackTheta(theta);

% Robot parameters
e = zeros(1,numParamsTotal);
e(calibBools) = e(calibBools) + x';

% System time offset
if ~fitTimeOffset
    tau = 0;
end

tOffset = t + tau;

qData = q(tOffset);
qDotData = qDot(tOffset);
qDDotData = qDDot(tOffset);

[alph, omeg] = ComputeImuMeasurements(qData, qDotData, qDDotData, e, g);

% Data scale factors
if ~fitScaleFactors
    s = ones(2,1);
end

alph = s(1).*alph;
omeg = s(2).*omeg;

y = [alph, omeg]';
y = y(:);
end