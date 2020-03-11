clear;
close all;

numParamsTotal = 8*6;

calibBools = true(1,numParamsTotal);
calibBools([8, 11, 15, 32, 36, 42, 46, 47, 48]) = false;

numParams = sum(calibBools);

paramsNominal = zeros(1,numParamsTotal);
xNominal = zeros(numParams,1);

xCovMag = (0.01)^2;
xCov = xCovMag*diag(ones(numParams,1));
xTruth = mvnrnd(xNominal, xCov)';

paramsTruth = paramsNominal;
paramsTruth(calibBools) = paramsTruth(calibBools) + xTruth';

% Measure robot position
numMeas = 1000;
pCovMag = 0.00000001^2;
pCovSingleMeas = pCovMag*eye(3);
pCov = pCovMag*eye(3*numMeas);
qCov = zeros(6);
showPlot = true;

jointLimits = [-pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4];
           
[p, q] = GenerateDiscreteMeasurements(numMeas, jointLimits, paramsTruth, pCovSingleMeas, qCov, showPlot);

% Regular least-squares estimate

H = ComputeIdJacobian(q, calibBools, false);
Z = computeResidual(xNominal, p, q, calibBools);

xStarLsq = ((H')*H) \ (H')*Z;

% Minimum-variance estimate

xStarMinVar = (inv((inv(xCov) + (H')*(inv(pCov))*H)))*(H')*(inv(pCov))*Z;

% Nonlinear least-squares estimate

obj = @(x) computeResidual(x, p, q, calibBools);

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'levenberg-marquardt', ...
                       'Display', 'iter');

xStarNlLsq = lsqnonlin(obj, xNominal, [], [], options);

table(xStarLsq, xStarMinVar, xStarNlLsq, xTruth)

function res = computeResidual(x, p, q, calibBools)
    params = zeros(1,8*6);
    params(calibBools) = params(calibBools) + x';
    
    numMeas = size(p,1);
    pHat = zeros(size(p));
    for iii = 1:numMeas
        pHat(iii,:) = ComputeForwardKinematics(q(iii,:), params, false);
    end
    
    resMatrix = (p - pHat)';
    res = resMatrix(:);
end