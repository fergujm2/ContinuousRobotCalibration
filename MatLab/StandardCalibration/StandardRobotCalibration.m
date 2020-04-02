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
numMeas = 600;
pCovMag = 0.00025^2;
pCov = pCovMag*eye(3);
pCovTotal = pCovMag*eye(3*numMeas);
qCov = zeros(6);
showPlot = true;

qTruth = SampleJointSpace(numMeas);
% qTruth = GetOptimalPoses(numMeas);
[p, q] = GenerateDiscreteMeasurements(qTruth, eTruth, pCov, qCov, showPlot);

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

xErrorLsq = abs(xStarLsq - xTruth);
xErrorMinVar = abs(xStarMinVar - xTruth);
xErrorNlLsq = abs(xStarNlLsq - xTruth);

eMeters = [ones(3,7); zeros(3,7)];
eMeters = logical(eMeters(:));

calibInd = find(calibBools); 
xMeters = eMeters(calibInd);
xRadians = not(xMeters);

maxDistanceErrorLsq = max(xErrorLsq(xMeters)); 
maxAngleErrorLsq = max(xErrorLsq(xRadians)); 
maxDistanceErrorMinVar = max(xErrorMinVar(xMeters)); 
maxAngleErrorMinVar = max(xErrorMinVar(xRadians)); 
maxDistanceErrorNlLsq = max(xErrorNlLsq(xMeters)); 
maxAngleErrorNlLsq = max(xErrorNlLsq(xRadians)); 

fprintf('Max Parameter Distance Error (mm):\n');
fprintf('  LSQ:              %.3f\n', maxDistanceErrorLsq*1e3);
fprintf('  Minimum Variance: %.3f\n', maxDistanceErrorMinVar*1e3);
fprintf('  Nonlinear LSQ:    %.3f\n', maxDistanceErrorNlLsq*1e3);

fprintf('Max Parameter Angle Error (deg):\n');
fprintf('  LSQ:              %.3f\n', maxAngleErrorLsq*180/pi);
fprintf('  Minimum Variance: %.3f\n', maxAngleErrorMinVar*180/pi);
fprintf('  Nonlinear LSQ:    %.3f\n', maxAngleErrorNlLsq*180/pi);


function res = computeResidual(x, p, q, calibBools)
    e = zeros(1,7*6);
    e(calibBools) = e(calibBools) + x';
    
    pHat = ComputeForwardKinematics(q, e, false);
    
    resMatrix = (p - pHat)';
    res = resMatrix(:);
end