clear;
close all;
rng(3);

% calibBools = logical([1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   0   0   0]);
% calibBools   = logical([0   0   0   0   0   0   0   0   0   0   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   1   1   1]);
calibBools     = logical([0   0   0   0   0   0   0   0   0   0   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   1   1   1]);

numParamsTotal = length(calibBools);
numParams = sum(calibBools);

eMeters = logical(reshape([ones(3, 7); zeros(3, 7)], 1, 42));
xMeters = eMeters(calibBools);
xRadians = not(xMeters);

xNominal = zeros(numParams,1);
xCov1 = (0.005)^2*eye(numParams - 6);
xCov2 = (0.1)^2*eye(6);
xCov = blkdiag(xCov1, xCov2);
xTruth = mvnrnd(xNominal, xCov)';

eNominal = zeros(1,numParamsTotal);
eTruth = eNominal;
eTruth(calibBools) = eTruth(calibBools) + xTruth';

gNominal = [0; 0; 9.81];
gCov = (0.01)^2*eye(3);
gTruth = mvnrnd(gNominal, gCov)';

tauNominal = 0;
tauCov = (0.01)^2;
tauTruth = mvnrnd(tauNominal, tauCov);

sNominal = [1; 1];
sCov = (1e-1)^2*eye(2);
sTruth = mvnrnd(sNominal, sCov)';

sampleRate = 30;
tSpan = [0, 180];
numWayPts = 100;

qCov = (0.1*pi/180)^2*eye(6);
alphCov = (0.01)^2*eye(3);
omegCov = (0.01)^2*eye(3);

showPlot = false;

qWayPts = SampleJointSpace(numWayPts);

[tData, qData, alphData, omegData] = GenerateImuMeasurements(qWayPts, sampleRate, tSpan, eTruth, gTruth, tauTruth, sTruth, qCov, alphCov, omegCov, showPlot);

d = 5;
[yq, Cq] = LsqFitVectorSpline(qData, tData(1), tData(end), d, floor(length(tData)./20));
[yqd, Cqd, dd] = DerVectorSpline(yq, Cq, d);
[yqdd, Cqdd, ddd] = DerVectorSpline(yqd, Cqd, dd);

q = @(t) EvalVectorSpline(yq, Cq, d, t);
qDot = @(t) EvalVectorSpline(yqd, Cqd, dd, t);
qDDot = @(t) EvalVectorSpline(yqdd, Cqdd, ddd, t);

qError = q(tData) - qData;
fprintf('\nmax(max(qError)): %f deg \n\n', max(max(qError))*180/pi); 

% Trim the data on the front and back
numMeas = length(tData);
numTrim = floor(numMeas*0.05);
tData = tData(numTrim:(end - numTrim));
qData = qData(numTrim:(end - numTrim),:);
alphData = alphData(numTrim:(end - numTrim),:);
omegData = omegData(numTrim:(end - numTrim),:);

theta0 = [xNominal; gNominal; tauNominal; sNominal];
thetaTruth = [xTruth; gTruth; tauTruth; sTruth];

thetaCov = blkdiag(xCov, gCov, tauCov, sCov);

thetaCovInv = inv(thetaCov);
alphCovInv = inv(alphCov);
omegCovInv = inv(omegCov);

obj = @(theta) computeImuObjective(theta, theta0, thetaCovInv, calibBools, tData, q, qDot, qDDot, alphData, alphCovInv, omegData, omegCovInv);

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

thetaStar = lsqnonlin(obj, theta0, [], [], options);

xStar = thetaStar(1:numParams);
gStar = thetaStar((numParams + 1):(numParams + 3));
tauStar = thetaStar(numParams + 4);
sStar = thetaStar((numParams + 5):end);

xErrors = abs(xStar - xTruth);
xErrorsMeters = xErrors(xMeters);
xErrorsRadians = xErrors(xRadians);

xErrorsMetersMax = max(xErrorsMeters);
xErrorsRadiansMax = max(xErrorsRadians);

fprintf('\n\nMaximum Estimation Errors: \n  Distance: %.6f mm \n     Angle: %.6f deg \n\n', xErrorsMetersMax*1000, xErrorsRadiansMax*180/pi);

figure(1);
clf;

subplot(5,1,1);
bar([xStar(1:(end - 6)), xTruth(1:(end - 6))]);
title('Robot Parameters');

subplot(5,1,2);
bar([xStar((end - 5):end), xTruth((end - 5):end)]);
title('IMU Offset Parameters');

subplot(5,1,3);
bar([gStar, gTruth]);
legend('Estimated', 'Truth');
title('Gravity Components');

subplot(5,1,4);
bar([tauStar, tauTruth], 'FaceColor', 'flat', 'CData', [0    0.4470    0.7410; 0.8500    0.3250    0.0980]);
title('Time Offset');

subplot(5,1,5);
bar([sStar, sTruth]);
title('Data Scale Factors');


function res = computeImuObjective(theta, theta0, thetaCovInv, calibBools, tData, q, qDot, qDDot, alphData, alphCovInv, omegData, omegCovInv)
    numParams = sum(calibBools);
    numParamsTotal = length(calibBools);
    
    % Robot parameters
    x = theta(1:numParams);
    e = zeros(1,numParamsTotal);
    e(calibBools) = e(calibBools) + x';
    
    % Gravity direction
    gw = theta((numParams + 1):(numParams + 3));
    
    % System time offset
    tau = theta(numParams + 4);
    t = tData + tau;
    
    qData = q(t);
    qDotData = qDot(t);
    qDDotData = qDDot(t);
    
    [alph, omeg] = ComputeImuMeasurements(qData, qDotData, qDDotData, e, gw);
    
    % Data scale factors
    s = theta((numParams + 5):end);
    alph = s(1).*alph;
    omeg = s(2).*omeg;
    
    ealph = (alph - alphData)';
% 	alphres = dot(ealph, alphCovInv*ealph);
    alphres = alphCovInv*ealph;
    
%     Jalph = 1/2*sum(alphres);

    eomeg = (omeg - omegData)';
% 	omegres = dot(eomeg, omegCovInv*eomeg);
    omegres = omegCovInv*eomeg;
    
    thetares = thetaCovInv*(theta - theta0);
    
    res = [alphres(:); omegres(:); thetares(:)];
%     res = [alphres(:); omegres(:)];
end

function J = computeJacobian(theta, obj)
    res0 = obj(theta);
    J = nan(length(res0), length(theta));  %pre-allocate
    
    del = 1e-9;
    delTheta = theta;
    
    for iii = 1:length(theta)
      delTheta(iii) = delTheta(iii) + del;
      J(:,iii) = (feval(obj, delTheta) - res0)/del;
      delTheta(iii) = theta(iii);
    end
end

