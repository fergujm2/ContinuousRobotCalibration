clear;
close all;

% calibBools = logical([1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   0   0   0]);
calibBools = false(1,42);
calibBools(12) = true;

numParamsTotal = length(calibBools);
numParams = sum(calibBools);

eNominal = zeros(1,numParamsTotal);
xNominal = zeros(numParams,1);

xCovMag = (0.001)^2;
xCov = xCovMag*diag(ones(numParams,1));
xTruth = mvnrnd(xNominal, xCov)';
eTruth = eNominal;
eTruth(calibBools) = eTruth(calibBools) + xTruth';

sampleRate = 10;
tSpan = [0, 100];
numWayPts = 30;
gTruth = [0, 0, -9.81]';

qCov = (0)^2*eye(6);
alphCov = (0)^2*eye(3);
omegCov = (0)^2*eye(3);

showPlot = false;
           
qWayPts = SampleJointSpace(numWayPts);
[tData, qData, alphData, omegData] = GenerateImuMeasurements(qWayPts, sampleRate, tSpan, eTruth, gTruth, qCov, alphCov, omegCov, showPlot);

[yqFit, CqFit] = LsqFitVectorQuinticSpline(qData, tData(1), tData(end));
q = @(t) EvalVectorQuinticSpline(yqFit, CqFit, t);

x0 = zeros(numParams, 1);
theta0 = [x0; gTruth];
% theta0 = [x0];

obj = @(theta) computeImuObjective(theta, calibBools, tData, q, alphData, omegData);

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'levenberg-marquardt', ...
                       'StepTolerance', 1e-12, ...
                       'Display', 'iter');
                   
thetaStar = lsqnonlin(obj, theta0, [], [], options);

function res = computeImuObjective(theta, calibBools, tData, q, alphData, omegData)
    numParams = sum(calibBools);
    numParamsTotal = length(calibBools);
    
    % Robot parameters
    x = theta(1:numParams);
    e = zeros(1,numParamsTotal);
    e(calibBools) = e(calibBools) + x';
    
    % Gravity direction
    gw = theta((numParams + 1):(numParams + 3));
    
%     eq = (qData - q(tData))';
%     eqW = qCovInv*eq;
%     gw = [0; 0; -9.81];
    [alph, omeg] = ComputeImuMeasurements(q, tData, e, gw);
    
    ealph = (alph - alphData)';
%     ealphW = alphCovInv*ealph;
    ealph = ealph(:,5:(end-5));
    
    eomeg = (omeg - omegData)';
%     eomegW = omegCovInv*eomeg;
    eomeg = eomeg(:,5:(end-5));
    res = [ealph(:); eomeg(:)];
end