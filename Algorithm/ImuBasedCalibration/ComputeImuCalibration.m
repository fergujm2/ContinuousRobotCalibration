function [thetaStar, CStar, y, d, JStar] = ComputeImuCalibration(tRobot, q, tImu, z)

[thetaNominal, thetaCov] = GetThetaNominal();

fprintf('Fitting joint values to spline functions.\n');

TRobot = tRobot(end) - tRobot(1);
knotsPerSecond = 3;
numInteriorKnots = floor(TRobot*knotsPerSecond);
d = 5;

[y, C0] = LsqFitVectorSpline(q, tRobot, d, numInteriorKnots);

qError = EvalVectorSpline(y, C0, d, tRobot) - q;
fprintf('\nmax(max(qError)): %f deg \n\n', max(max(abs(qError)))*180/pi); 

% Trim the data on the front and back
tTrim = 1; % sec
numTrim = sum((tImu - tImu(1)) < tTrim);
tImu = tImu(numTrim:(end - numTrim));
z = z(numTrim:(end - numTrim),:);

[zCov, qCov] = GetCovariances();
numMeasZ = size(z,1);
numMeasQ = size(q,1);

Re = [repmat(qCov, numMeasQ, 1); repmat(zCov, numMeasZ, 1); diag(thetaCov)];
Ce = sqrt(1./Re);

obj = @(params) ComputeImuObjective(params, tRobot, q, y, d, tImu, z, thetaNominal, Ce);

JSparsity = GetJacobianSparsity(tRobot, tImu, y, d, C0);

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'trust-region-reflective', ...
                       'StepTolerance', 1e-8, ...
                       'FunctionTolerance', 1e-8, ...
                       'Display', 'iter', ...
                       'UseParallel', true, ...
                       'JacobPattern', JSparsity);
                   
paramsNominal = [thetaNominal; C0(:)];

fprintf('Computing maximum likelihood estimate of theta.\n');
[paramsStar, ~, ~, ~, ~, ~, JStar] = lsqnonlin(obj, paramsNominal, [], [], options);

thetaStar = paramsStar(1:length(thetaNominal));
CStar = reshape(paramsStar((length(thetaNominal) + 1):end), 6, []);

% Now plot measured data with the new theta
% zMeasStar = ImuMeasurementEquation(thetaStar, tImu, qf, qDot, qDDot);
% PlotImuMeasurements(tRobot, q, qf(tRobot), tImu, z, zMeasStar);
% drawnow();

end


