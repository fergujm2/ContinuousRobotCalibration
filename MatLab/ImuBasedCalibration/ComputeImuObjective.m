function res = ComputeImuObjective(theta, t, q, qDot, qDDot, z, zCovInv)

alphData = z(:,1:3);
omegData = z(:,4:6);
alphCovInv = zCovInv(1:3,1:3);
omegCovInv = zCovInv(4:6,4:6);

[calibBools, numParams, numParamsTotal] = GetCalibInfo();

% Robot parameters
x = theta(1:numParams);
e = zeros(1,numParamsTotal);
e(calibBools) = e(calibBools) + x';

% Gravity direction
gw = theta((numParams + 1):(numParams + 3));

% System time offset
tau = theta(numParams + 4);
tOffset = t + tau;

qData = q(tOffset);
qDotData = qDot(tOffset);
qDDotData = qDDot(tOffset);

[alph, omeg] = ComputeImuMeasurements(qData, qDotData, qDDotData, e, gw);

% Data scale factors
s = theta((numParams + 5):end);
alph = s(1).*alph;
omeg = s(2).*omeg;

ealph = (alph - alphData)';
alphres = sqrt(alphCovInv)*ealph;

eomeg = (omeg - omegData)';
omegres = sqrt(omegCovInv)*eomeg;

res = [alphres(:); omegres(:)];

end