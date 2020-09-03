function [thetaStar, thetaStarCov] = ComputeImuCalibration(tRobot, q, tImu, z, thetaNominal)

fprintf('Fitting joint values to spline functions.\n');

TRobot = tRobot(end) - tRobot(1);
knotsPerSecond = 5;
numInteriorKnots = floor(TRobot*knotsPerSecond);

[qf, qDot, qDDot] = GetVectorSplineFunctions(q, tRobot, 5, numInteriorKnots);

qError = qf(tRobot) - q;
fprintf('\nmax(max(qError)): %f deg \n\n', max(max(qError))*180/pi); 

% Trim the data on the front and back
tTrim = 1; % sec
numTrim = sum((tImu - tImu(1)) < tTrim);
tImu = tImu(numTrim:(end - numTrim));
z = z(numTrim:(end - numTrim),:);

% Now we need to determine the full covariance of our measurements
measCov = GetMeasurementCovariance(size(z, 1));
measCovInv = inv(measCov);
sqrtMeasCovInv = sqrt(measCovInv);

obj = @(theta) ComputeImuObjective(theta, qf, qDot, qDDot, tImu, z, sqrtMeasCovInv);

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'levenberg-marquardt', ...
                       'StepTolerance', 1e-8, ...
                       'FunctionTolerance', 1e-8, ...
                       'Display', 'iter', ...
                       'UseParallel', true);

fprintf('Computing maximum likelihood estimate of theta.\n');
[thetaStar, ~, ~, ~, ~, ~, JStar] = lsqnonlin(obj, thetaNominal, [], [], options);

% Linear approximation of error propogation
thetaStarCov = inv((JStar')*measCovInv*JStar);

% Now plot measured data with the new theta
zStar = ImuMeasurementEquation(thetaStar, tImu, qf, qDot, qDDot);
PlotImuMeasurements(tRobot, q, qf(tRobot), tImu, z, zStar);
drawnow();

end


