function [thetaStar, thetaStarCov] = CalibrateLeastSquares(t, q, z, zCov, thetaNominal, thetaCov)

d = 5;
[yq, Cq] = LsqFitVectorSpline(q, t(1), t(end), d, floor(length(t)./10), 1e-6);
[yqd, Cqd, dd] = DerVectorSpline(yq, Cq, d);
[yqdd, Cqdd, ddd] = DerVectorSpline(yqd, Cqd, dd);

qf = @(t) EvalVectorSpline(yq, Cq, d, t);
qDot = @(t) EvalVectorSpline(yqd, Cqd, dd, t);
qDDot = @(t) EvalVectorSpline(yqdd, Cqdd, ddd, t);

% Trim the data on the front and back
numMeas = length(t);
numTrim = floor(numMeas*0.05);
t = t(numTrim:(end - numTrim));
q = q(numTrim:(end - numTrim),:);
z = z(numTrim:(end - numTrim),:);

qError = qf(t) - q;
fprintf('\nmax(max(qError)): %f deg \n\n', max(max(qError))*180/pi); 

zDataCov = diag(zCov)*ones(1, size(z, 1));
measCov = zDataCov(:);
measCov = spdiags(measCov, 0, length(measCov), length(measCov));

zCovInv = inv(zCov);

obj = @(theta) ComputeImuObjective(theta, t, qf, qDot, qDDot, z, zCovInv);

% JTruth = computeJacobian(thetaTruth, obj);
% JNominal = computeJacobian(theta0, obj);
% cTruth = cond(JTruth)
% cNominal = cond(JNominal)

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'levenberg-marquardt', ...
                       'StepTolerance', 1e-14, ...
                       'MaxFunctionEvaluations', 3e2, ...
                       'Display', 'iter', ...
                       'UseParallel', true);

[thetaStar, ~, ~, ~, ~, ~, JStar] = lsqnonlin(obj, thetaNominal, [], [], options);

% Note that I need to check how measCov is built from zCov.
thetaStarCov = inv((JStar')*inv(measCov)*JStar);

end