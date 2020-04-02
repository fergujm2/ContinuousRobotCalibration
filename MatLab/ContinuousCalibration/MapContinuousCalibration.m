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

dCov = 0.025^2;
dTruth = sqrt(dCov)*randn();

% Measure robot position
sampleRate = 30;
tSpan = [0, 120];
numWayPts = 30;
% pCovMag = 0.00025^2;
pCovMag = 0.001^2;
pCov = pCovMag*eye(3);
qCovMag = 0^2;
qCov = qCovMag*eye(6);
showPlot = false;
           
qWayPts = SampleJointSpace(numWayPts);
[p, qData, t] = GenerateContinuousMeasurements(qWayPts, sampleRate, tSpan, dTruth, eTruth, pCov, qCov, showPlot);

% Now we have access to the joint space function q(t)
[yq, Cq] = LsqFitVectorQuinticSpline(qData, t(1), t(end));
q = @(t) EvalVectorQuinticSpline(yq, Cq, t);

theta0 = [0; zeros(numParams,1)];
thetaCov = blkdiag(dCov, xCov);

thetaCovInv = inv(thetaCov);
pCovInv = inv(pCov);

% obj = @(theta, showPlot) computeMapResidual(theta, thetaCovInv, p, pCovInv, q, t, calibBools, showPlot);
% 
% opts = optimoptions(@fminunc, ...
%                     'Display', 'iter', ...
%                     'OptimalityTolerance', 1e-14, ...
%                     'StepTolerance', 1e-10, ...
%                     'MaxFunctionEvaluations', 1e4, ...
%                     'UseParallel', true);
%                 
% thetaStar = fminunc(@(theta) obj(theta, false), theta0, opts);

obj = @(theta, showPlot) computeLsqResidual(theta, p, q, t, calibBools, showPlot);

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'levenberg-marquardt', ...
                       'Display', 'iter');
                   
thetaStar = lsqnonlin(@(theta) obj(theta, false), theta0, [], [], options);

thetaTruth = [dTruth; xTruth];

res0 = obj(theta0, true);
resFinal = obj(thetaStar, true);

numRobotParams = length(xTruth);
paramNames = cell(length(thetaTruth), 1);
paramNames{1} = 'Time Offset';

for iii = 2:length(thetaTruth)
    paramNames{iii} = sprintf('Robot Parameter %d', iii - 1);
end

thetaError = abs(thetaStar - thetaTruth);
table(paramNames, thetaTruth, thetaStar, thetaError)

dError = thetaError(1);
xError = thetaError(2:end);

eMeters = [ones(3,7); zeros(3,7)];
eMeters = logical(eMeters(:));

calibInd = find(calibBools); 
xMeters = eMeters(calibInd);
xRadians = not(xMeters);

maxDistanceError = max(xError(xMeters)); 
maxAngleError = max(xError(xRadians));

fprintf('Time Offset Error (ms):            %.3f\n', dError*1e3);
fprintf('Max Parameter Distance Error (mm): %.3f\n', maxDistanceError*1e3);
fprintf('Max Parameter Angle Error (deg):   %.3f\n', maxAngleError*180/pi);


function res = computeMapResidual(theta, thetaCovInv, p, pCovInv, q, t, calibBools, showPlot)
    d = theta(1);
    
    % Trim t
    indTrim = and(t > (t(1) + .15), t < (t(end) - .15));
    t = t(indTrim);
    p = p(indTrim,:);
    
    qt = q(t + d);
    
    x = theta(2:end);
    e = zeros(1,length(calibBools));
    e(calibBools) = e(calibBools) + x';

    pHat = ComputeForwardKinematics(qt, e, false);
    
    if showPlot
        figure();
        clf;
        hold on;
        
        ComputeForwardKinematics(zeros(1,6), e, true);
        plot3(p(:,1), p(:,2), p(:,3), '.-k');
        plot3(pHat(:,1), pHat(:,2), pHat(:,3), '.-r');
    
        grid on;
        daspect([1,1,1]);
        view([30,30]);
        drawnow;
    end
    
    thetaMu = zeros(size(theta)); % mean of zero right now
    eTheta = theta - thetaMu;
    JTheta = 1/2*(eTheta')*thetaCovInv*eTheta;
    
    eZ = (p - pHat)';
    JZ = sum(dot(eZ, pCovInv*eZ));
    
    res = JTheta + JZ;
end

function res = computeLsqResidual(theta, p, q, t, calibBools, showPlot)
    d = theta(1);
    
    % Trim t
    indTrim = and(t > (t(1) + .15), t < (t(end) - .15));
    t = t(indTrim);
    p = p(indTrim,:);
    
    qt = q(t + d);
    
    x = theta(2:end);
    e = zeros(1,length(calibBools));
    e(calibBools) = e(calibBools) + x';

    pHat = ComputeForwardKinematics(qt, e, false);
    
    if showPlot
        figure();
        clf;
        hold on;
        
        ComputeForwardKinematics(zeros(1,6), e, true);
        plot3(p(:,1), p(:,2), p(:,3), '.-k');
        plot3(pHat(:,1), pHat(:,2), pHat(:,3), '.-r');
    
        grid on;
        daspect([1,1,1]);
        view([30,30]);
        drawnow;
    end
    
    resMatrix = (p - pHat)';
    res = resMatrix(:);
end