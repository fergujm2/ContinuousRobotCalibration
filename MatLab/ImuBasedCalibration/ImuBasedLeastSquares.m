clear;
close all;

% calibBools = logical([1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   0   0   0]);
calibBools = false(1,42);

% calibBools(8) = true;
% calibBools(10) = true;
% calibBools(12) = true;
% calibBools(25) = true;
% calibBools(12) = true;
% calibBools(15) = true;
% calibBools(26) = true;
calibBools((end - 5):end) = true;

numParamsTotal = length(calibBools);
numParams = sum(calibBools);

xNominal = zeros(numParams,1);
xCov = (0.001)^2*eye(numParams);
xTruth = mvnrnd(xNominal, xCov)';

eNominal = zeros(1,numParamsTotal);
eTruth = eNominal;
eTruth(calibBools) = eTruth(calibBools) + xTruth';

gNominal = [0; 0; 9.81];
gCov = (0.01)^2*eye(3);
gTruth = mvnrnd(gNominal, gCov)';

sampleRate = 60;
tSpan = [0, 120];
numWayPts = 10;

qCov = (0.01*pi/180)^2*eye(6);
alphCov = (0.01)^2*eye(3);
omegCov = (0.01)^2*eye(3);

Qa = (0.01)^2*eye(3);
Qw = (0.01)^2*eye(3);

showPlot = false;

qWayPts = SampleJointSpace(numWayPts);
[tData, qData, alphData, omegData, yb, db, Cba, Cbw] = GenerateImuMeasurements(qWayPts, sampleRate, tSpan, eTruth, gTruth, qCov, alphCov, omegCov, Qa, Qw, showPlot);

d = 5;
[yq, Cq] = LsqFitVectorSpline(qData, tData(1), tData(end), d, 50);
[yqd, Cqd, dd] = DerVectorSpline(yq, Cq, d);
[yqdd, Cqdd, ddd] = DerVectorSpline(yqd, Cqd, dd);

qData = EvalVectorSpline(yq, Cq, d, tData);
qDotData = EvalVectorSpline(yqd, Cqd, dd, tData);
qDDotData = EvalVectorSpline(yqdd, Cqdd, ddd, tData);

Cba0 = Cba;
Cbw0 = Cbw;
cba0 = Cba0(:);
cbw0 = Cbw0(:);

theta0 = [xNominal; gNominal; zeros(size(cba0)); zeros(size(cbw0))];

% qCovInv = inv(qCov);
alphCovInv = inv(alphCov);
omegCovInv = inv(omegCov);
QaInv = inv(Qa);
QwInv = inv(Qw);

obj = @(theta) computeImuObjective(theta, calibBools, tData, qData, qDotData, qDDotData, alphData, alphCovInv, omegData, omegCovInv, yb, db, QaInv, QwInv);

% options = optimoptions(@lsqnonlin, ...
%                        'Algorithm', 'levenberg-marquardt', ...
%                        'StepTolerance', 1e-12, ...
%                        'Display', 'iter');
% 
% thetaStar = lsqnonlin(obj, theta0, [], [], options);

opts = optimoptions(@fminunc, ...
                    'Display', 'iter', ...
                    'OptimalityTolerance', 1e-14, ...
                    'StepTolerance', 1e-10, ...
                    'MaxFunctionEvaluations', 1e5);
                
thetaStar = fminunc(obj, theta0, opts);

xStar = thetaStar(1:numParams);
gStar = thetaStar((numParams + 1):(numParams + 3));
cbStar = thetaStar((numParams + 4):end);

cbaStar = cbStar(1:(length(cbStar)/2));
cbwStar = cbStar((length(cbStar)/2 + 1):end);
CbaStar = reshape(cbaStar, 3, length(cbaStar)/3);
CbwStar = reshape(cbwStar, 3, length(cbwStar)/3);

alphBias = EvalVectorSpline(yb, CbaStar, db, tData);
alphBiasTruth = EvalVectorSpline(yb, Cba, db, tData);
omegBias = EvalVectorSpline(yb, CbwStar, db, tData);
omegBiasTruth = EvalVectorSpline(yb, Cbw, db, tData);

figure(1);
clf;

subplot(2,1,1);
bar([xStar, xTruth]);
legend('xStar', 'xTruth');
title('Robot Parameter Errors');

subplot(2,1,2);
bar([gStar, gTruth]);
legend('gStar', 'gTruth');
title('Gravity Direction Errors');

figure(2);

subplot(2,1,1);
hold on;
plot(tData, alphBias, '-');
plot(tData, alphBiasTruth, '-.');

subplot(2,1,2);
hold on;
plot(tData, omegBias, '-');
plot(tData, omegBiasTruth, '-.');


function res = computeImuObjective(theta, calibBools, tData, qData, qDotData, qDDotData, alphData, alphCovInv, omegData, omegCovInv, yb, db, QaInv, QwInv)
    numParams = sum(calibBools);
    numParamsTotal = length(calibBools);
    
    % Robot parameters
    x = theta(1:numParams);
    e = zeros(1,numParamsTotal);
    e(calibBools) = e(calibBools) + x';
    
    % Gravity direction
    gw = theta((numParams + 1):(numParams + 3));
    
%     cq = theta((numParams + 4):end);
%     Cq = reshape(cq, 6, length(cq)/6);
%     d = 5;
%     [yqd, Cqd, dd] = DerVectorSpline(yq, Cq, d);
%     [yqdd, Cqdd, ddd] = DerVectorSpline(yqd, Cqd, dd);
% 
%     q = EvalVectorSpline(yq, Cq, d, tData);
%     qDot = EvalVectorSpline(yqd, Cqd, dd, tData);
%     qDDot = EvalVectorSpline(yqdd, Cqdd, ddd, tData);
%     
%     eq = (qData - q)';
%     eqW = qCovInv*eq;
    
    [alph, omeg] = ComputeImuMeasurements(qData, qDotData, qDDotData, e, gw);
    
    cb = theta((numParams + 4):end);
    cba = cb(1:(length(cb)/2));
    cbw = cb((length(cb)/2 + 1):end);
    Cba = reshape(cba, 3, length(cba)/3);
    Cbw = reshape(cbw, 3, length(cbw)/3);
    
    alphBias = EvalVectorSpline(yb, Cba, db, tData);
    omegBias = EvalVectorSpline(yb, Cbw, db, tData);
    
    ealph = (alph + alphBias - alphData)';
%     ealph = ealph(:,5:(end-5));
    Jalph = 1/2*sum(dot(ealph, alphCovInv*ealph));

    eomeg = (omeg + omegBias - omegData)';
%     eomeg = eomeg(:,5:(end-5));
    Jomeg = 1/2*sum(dot(eomeg, omegCovInv*eomeg));
    
    [ybd, Cbad, dbd] = DerVectorSpline(yb, Cba, db);
    [~, Cbwd, ~] = DerVectorSpline(yb, Cbw, db);
    
    baDot = EvalVectorSpline(ybd, Cbad, dbd, tData)';
    bwDot = EvalVectorSpline(ybd, Cbwd, dbd, tData)';
    
    inta = trapz(tData, dot(baDot, QaInv*baDot));
    intw = trapz(tData, dot(bwDot, QwInv*bwDot));
    
    Jba = 1/2*inta;
    Jbw = 1/2*intw;
    
    res = Jalph + Jomeg + Jba + Jbw;
end