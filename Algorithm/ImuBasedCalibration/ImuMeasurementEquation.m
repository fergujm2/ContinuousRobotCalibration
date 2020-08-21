function z = ImuMeasurementEquation(theta, t, q, qDot, qDDot)

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
[fitTimeOffset, fitScaleFactors, fitXyzOffsets] = GetCalibOptions();

[x, g, tau, s, b] = UnpackTheta(theta);

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

z = ComputeImuMeasurements(qData, qDotData, qDDotData, e, g);

% Imu data scale factors
if ~fitScaleFactors
    s = ones(2,1);
end

% Imu data offsets
if ~fitXyzOffsets
    b = zeros(6,1);
end

z = [s(1).*z(:,1:3), s(2).*z(:,4:6)] + ones(size(z,1),1)*(b');
end