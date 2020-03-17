clear;
close all;

calibBools = logical([1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   0   0   0]);
numParamsTotal = length(calibBools);
numParams = sum(calibBools);

eNominal = zeros(1,numParamsTotal);
xNominal = zeros(numParams,1);

xCovMag = (0.01)^2;
xCov = xCovMag*diag(ones(numParams,1));
xTruth = mvnrnd(xNominal, xCov)';

eTruth = eNominal;
eTruth(calibBools) = eTruth(calibBools) + xTruth';

% Measure robot position
numMeas = 30;
pCovMag = 0.00025^2;
pCov = pCovMag*eye(3);
pCovTotal = pCovMag*eye(3*numMeas);
qCov = zeros(6);
showPlot = true;

jointLimits = [-pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4];
           
[p, q] = GenerateDiscreteMeasurements(numMeas, jointLimits, eTruth, pCov, qCov, showPlot);

% Regular least-squares estimate

H = ComputeIdJacobian(q, calibBools, false);
Z = computeResidual(xNominal, p, q, calibBools);

xStarLsq = ((H')*H) \ (H')*Z;

% Minimum-variance estimate

xStarMinVar = (inv((inv(xCov) + (H')*(inv(pCovTotal))*H)))*(H')*(inv(pCovTotal))*Z;

% Nonlinear least-squares estimate

obj = @(x) computeResidual(x, p, q, calibBools);

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'levenberg-marquardt', ...
                       'Display', 'iter');

xStarNlLsq = lsqnonlin(obj, xNominal, [], [], options);

table(xStarLsq, xStarMinVar, xStarNlLsq, xTruth)

lsqMaxNorm = max(xStarLsq - xTruth);
minVarMaxNorm = max(xStarMinVar - xTruth);
nlLsqMaxNorm = max(xStarNlLsq - xTruth);

table(lsqMaxNorm, minVarMaxNorm, nlLsqMaxNorm)

function res = computeResidual(x, p, q, calibBools)
    params = zeros(1,7*6);
    params(calibBools) = params(calibBools) + x';
    
    numMeas = size(p,1);
    pHat = zeros(size(p));
    for iii = 1:numMeas
        pHat(iii,:) = ComputeForwardKinematics(q(iii,:), params, false);
    end
    
    resMatrix = (p - pHat)';
    res = resMatrix(:);
end