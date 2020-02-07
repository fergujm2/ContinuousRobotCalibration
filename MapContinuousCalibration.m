clear;
close all;

robot = GetRobot();

dhParamsNominal = GetDhParameters(robot);

calibBools = false(size(dhParamsNominal));

calibBools(1,:) = true;
calibBools(3,:) = true;

numParams = sum(sum(calibBools));

robotParamsCovMag = 0.1^2;
robotParamsCov = robotParamsCovMag*eye(numParams);
robotParamsTruth = mvnrnd(zeros(numParams,1), robotParamsCov, 1)';

dhParamsTruth = dhParamsNominal;
dhParamsTruth(calibBools) = dhParamsNominal(calibBools) + robotParamsTruth;

dCov = 0.01^2;
dTruth = sqrt(dCov)*randn();

% Measure robot position
numMeasurements = 600;
numWayPts = 15;
a = 0;
b = 1;
pCovMag = 0.01^2;
pCov = pCovMag*eye(3);
qCovMag = 0.00^2;
qCov = qCovMag*eye(6);
showPlot = false;

robot = SetDhParameters(robot, dhParamsTruth);
[p, qData, t] = GenerateContinuousMeasurements(robot, numMeasurements, numWayPts, a, b, 'pCov', pCov, 'qCov', qCov, 'tau', dTruth, 'ShowPlot', showPlot);
robot = SetDhParameters(robot, dhParamsNominal);

% Now we have access to the joint space function q(t)
[yq, Cq] = LsqFitVectorQuinticSpline(qData, t(1), t(end));
q = @(t) EvalVectorQuinticSpline(yq, Cq, t);

theta0 = [0; zeros(numParams,1)];
thetaCov = blkdiag(dCov, robotParamsCov);

thetaCovInv = inv(thetaCov);
pCovInv = inv(pCov);

obj = @(theta, showPlot) computeObjective(theta, thetaCovInv, robot, p, pCovInv, q, t, dhParamsNominal, calibBools, showPlot);

opts = optimoptions(@fminunc, ...
                    'Display', 'iter', ...
                    'OptimalityTolerance', 1e-14, ...
                    'StepTolerance', 1e-10, ...
                    'SpecifyObjectiveGradient', true);
                
thetaStar = fminunc(@(theta) obj(theta, false), theta0, opts);

thetaTruth = [dTruth; robotParamsTruth];

res0 = obj(theta0, true);
resFinal = obj(thetaStar, true);

numRobotParams = length(robotParamsTruth);
paramNames = cell(length(thetaTruth), 1);
paramNames{1} = 'Time Offset';

for iii = 2:length(thetaTruth)
    paramNames{iii} = sprintf('Robot Parameter %d', iii - 1);
end

error = abs(thetaStar - thetaTruth);
table(paramNames, thetaTruth, thetaStar, error)

function res = computeResidual(theta, thetaCovInv, robot, p, pCovInv, q, t, dhParamsNominal, calibBools, showPlot)
    d = theta(1);
    robotParams = theta(2:end);
    
    dhParams = dhParamsNominal;
    dhParams(calibBools) = dhParams(calibBools) + robotParams;
    
    robot = SetDhParameters(robot, dhParams);
    
    homeConfig = robot.homeConfiguration();
    numJoints = length(homeConfig);
    
    % Trim t
    indTrim = and(t > 0.15, t < 0.85);
    t = t(indTrim);
    p = p(indTrim,:);
    
    qt = q(t + d);
    numMeas = length(t);
    pHat = zeros(numMeas, 3);
    
    for iii = 1:numMeas
        qConfig = homeConfig;
        for jjj = 1:numJoints
            qConfig(jjj).JointPosition = qt(iii,jjj);
        end
        Ti = robot.getTransform(qConfig, robot.BodyNames{end});
        pHat(iii,:) = Ti(1:3,4);
    end
    
    if showPlot
        figure();
        clf;
        hold on;
        scatter3(p(:,1), p(:,2), p(:,3), 20, 'MarkerEdgeColor', 'black');
        scatter3(pHat(:,1), pHat(:,2), pHat(:,3), 20, 'Filled', 'MarkerFaceColor', 'red');
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

function [f, g] = computeObjective(theta, thetaCovInv, robot, p, pCovInv, q, t, dhParamsNominal, calibBools, showPlot)
    f = computeResidual(theta, thetaCovInv, robot, p, pCovInv, q, t, dhParamsNominal, calibBools, showPlot);
    
    del = 1e-10;
    
    n = length(theta);
    g = zeros(n, 1);
    
    for iii = 1:n
        theta1 = theta;
        theta2 = theta;
        
        theta2(iii) = theta2(iii) + del;
        theta1(iii) = theta1(iii) - del;
        
        f2 = computeResidual(theta2, thetaCovInv, robot, p, pCovInv, q, t, dhParamsNominal, calibBools, false);
        f1 = computeResidual(theta1, thetaCovInv, robot, p, pCovInv, q, t, dhParamsNominal, calibBools, false);
        
        g(iii) = (f2 - f1)/2/del;
    end
end



