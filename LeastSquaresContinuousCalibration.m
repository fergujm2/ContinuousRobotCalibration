clear;
close all;

robot = GetRobot();

dhParamsNominal = GetDhParameters(robot);

calibBools = false(size(dhParamsNominal));

calibBools(1,2) = true;
calibBools(3,1) = true;
calibBools(1,1) = true;

numParams = sum(sum(calibBools));

robotParamsCov = 0.1*eye(numParams);
robotParamsTruth = mvnrnd(zeros(numParams,1), robotParamsCov, 1)';

dhParamsTruth = dhParamsNominal;
dhParamsTruth(calibBools) = dhParamsNominal(calibBools) + robotParamsTruth;

% Measure robot position
numMeasurements = 1000;
numWayPts = 7;
a = 0;
b = 1;
pCovMag = 0.01^2;
pCov = pCovMag*eye(3);
qCovMag = 0.001^2;
qCov = qCovMag*eye(6);
tau = 0.05;
showPlot = false;

robot = SetDhParameters(robot, dhParamsTruth);
[p, q, t] = GenerateContinuousMeasurements(robot, numMeasurements, numWayPts, a, b, 'pCov', pCov, 'qCov', qCov, 'tau', tau, 'ShowPlot', showPlot);
robot = SetDhParameters(robot, dhParamsNominal);

% Now we assume that the measurements p have been taken at a time offset
% from the measurements at q and also have noise as well.
x0 = [0; zeros(numParams,1)];
resInit = computeScalarResidual(x0, robot, p, q, t, dhParamsNominal, calibBools, true);

obj = @(x) computeScalarResidual(x, robot, p, q, t, dhParamsNominal, calibBools, false);

opts = optimoptions(@fminunc, 'Display', 'iter');
xStar = fminunc(obj, x0, opts);

% opts = optimset('Display', 'Iter');
% xStar = fminsearch(obj, x0, opts);

xTruth = [tau; robotParamsTruth];

resFinal = computeScalarResidual(xStar, robot, p, q, t, dhParamsNominal, calibBools, true);



function res = computeScalarResidual(x, robot, p, q, t, dhParamsNominal, calibBools, showPlot)
    tau = x(1);
    robotParams = x(2:end);
    
    dhParams = dhParamsNominal;
    dhParams(calibBools) = dhParams(calibBools) + robotParams;
    
    robot = SetDhParameters(robot, dhParams);
    
    numMeas = size(p, 1);
    
    pHat = zeros(numMeas, 3);
    
    homeConfig = robot.homeConfiguration();
    numJoints = length(homeConfig);
    
    for iii = 1:numMeas
        qConfig = homeConfig;
        
        for jjj = 1:numJoints
            qConfig(jjj).JointPosition = q(iii,jjj);
        end
        
        Ti = robot.getTransform(qConfig, robot.BodyNames{end});
        pHat(iii,:) = Ti(1:3,4);
    end
    
    % Now we need to fit both p and pHat to splines to integrate error
    tHat = t - tau;
    [yp, Cp] = LsqFitVectorQuinticSpline(p, t(1), t(end));
    [ypHat, CpHat] = LsqFitVectorQuinticSpline(pHat, tHat(1), tHat(end));
    
    error = @(t) sum((EvalVectorQuinticSpline(yp, Cp, t) - EvalVectorQuinticSpline(ypHat, CpHat, t)).^2, 2)';
    
    if showPlot
        tPlot = linspace(t(1), t(end), 10000);
        tHatPlot = linspace(tHat(1), tHat(end), 10000);
        
        pPlot = EvalVectorQuinticSpline(yp, Cp, tPlot);
        pHatPlot = EvalVectorQuinticSpline(ypHat, CpHat, tHatPlot);
        
        figure();
        clf;
        subplot(1, 2, 1);
        hold on;
        plot3(pPlot(:,1), pPlot(:,2), pPlot(:,3), 'red', 'LineWidth', 1);
        plot3(pHatPlot(:,1), pHatPlot(:,2), pHatPlot(:,3), 'blue', 'LineWidth', 1);
        grid on;
        daspect([1,1,1]);
        view([30,30]);
        drawnow;
        
        subplot(1, 2, 2);
        fplot(error, [0.1, 0.9]);
        drawnow();
    end
    
    
    res = integral(error, 0.1, 0.9);
end
