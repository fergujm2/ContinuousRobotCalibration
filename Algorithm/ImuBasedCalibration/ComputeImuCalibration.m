function [thetaStar, thetaStarCov] = ComputeImuCalibration(tRobot, q, qCov, qDotCov, qDDotCov, tImu, z, zCov, thetaNominal, thetaCov)

fprintf('Fitting joint values to spline functions.\n');

[qf, qDot, qDDot] = GetVectorSplineFunctions(q, tRobot, 5, floor(length(tRobot)./30));

qError = qf(tRobot) - q;
fprintf('\nmax(max(qError)): %f deg \n\n', max(max(qError))*180/pi); 

% Trim the data on the front and back
tTrim = 1; % sec
numTrim = sum((tImu - tImu(1)) < tTrim);
tImu = tImu(numTrim:(end - numTrim));
z = z(numTrim:(end - numTrim),:);

% Now we need to determine the full covariance of our measurements
fprintf('Computing measurement covariance matrix.\n');
measCov = ComputeMeasurementCovariance(qf(tImu), qDot(tImu), qDDot(tImu), thetaNominal, zCov, qCov, qDotCov, qDDotCov);
measCovInv = inv(measCov);
[cholMeasCovInv, p] = chol(measCovInv);

if p ~= 0
    warning('Inverse of measurement covariance matrix is not positive definite.');
end

obj = @(theta) ComputeImuObjective(theta, tImu, qf, qDot, qDDot, z, cholMeasCovInv);

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


