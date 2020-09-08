function ComputeOptimalTrajectory()

close all;

jointLimits = GetJointLimits();
numJoints = size(jointLimits, 1);

T = 5;
sampleRate = 100;
tSpan = [0, 1*T];
n = 2;

obj = @(AB) computeObservabilityMeasure(AB, T, sampleRate, tSpan);
% con = @(AB) nonlinearConstraint(AB, T);

AB0 = GetRandomAB(n, numJoints, T);

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.DisplayInterval = 1;

customPlotFcn = @(x, optimvalues, flag) plotBestFunctionsSa(x, optimvalues, flag, sampleRate, tSpan, T);
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf,customPlotFcn};
options.PlotInterval = 10;
options.MaxIterations = 10000;
options.AnnealingFcn = @(optimValues,problem) AnnealingJointLimits(optimValues,problem,T);
options.InitialTemperature = 50;

bound = 2.*ones(size(AB0));
LB = -bound;
UB = bound;

AB = simulannealbnd(obj, AB0, LB, UB, options);

% Refine solution with general constrainted optimization
options = optimoptions('patternsearch');
options.Display = 'iter';
options.MaxIterations = 300;
customPlotFcn = @(x, flag) plotBestFunctionsPs(x, flag, T, sampleRate, tSpan);
options.InitialMeshSize = 1;
options.MaxMeshSize = 1;
options.UseParallel = true;
options.PlotFcn = {@psplotbestf, @psplotmeshsize, customPlotFcn};
options.PollMethod = 'GPSPositiveBasisNp1';
options.UseCompletePoll = true;
options.Cache = 'on';

AB = patternsearch(@(AB) objPs(AB, T, sampleRate, tSpan), AB, [], [], [], [], [], [], [], options);

[~, thetaCov] = obj(AB);

A = AB(1:(n + 1),:);
B = AB((n + 2):end,:);

calibBools = GetRobotCalibInfo();

filename = fullfile('Output', 'OptimalTrajectory.mat');
save(filename, 'A', 'B', 'T', 'sampleRate', 'thetaCov', 'calibBools');

end

function y = objPs(AB, T, sampleRate, tSpan)
    if ~CheckJointLimits(AB, T)
        y = 1e6;
    else
        y = computeObservabilityMeasure(AB, T, sampleRate, tSpan);
    end
end

function [y, thetaCov] = computeObservabilityMeasure(AB, T, sampleRate, tSpan)
    theta = GetThetaNominal();
    
    numMeas = sampleRate*(tSpan(2) - tSpan(1));
    t = linspace(tSpan(1), tSpan(2), numMeas);
    
    n = (size(AB, 1) - 1)/2;
    A = AB(1:n + 1,:);
    B = AB(n + 2:end,:);
    
    [Ad, Bd] = DerVectorFourier(A, B, T);
    [Add, Bdd] = DerVectorFourier(Ad, Bd, T);
    
    q = @(t) EvalVectorFourier(A, B, t, T);
    qDot = @(t) EvalVectorFourier(Ad, Bd, t, T);
    qDDot = @(t) EvalVectorFourier(Add, Bdd, t, T);
    
    z = zeros(length(t), 6);
    
    measCov = GetMeasurementCovariance(numMeas);
    
    % Now we need the covariance to be identity to make sure we're using
    % the correct Jacobian.
    sqrtMeasCovInv = measCov \ measCov;
    
    obj = @(theta) ComputeImuObjective(theta, q, qDot, qDDot, t, z, sqrtMeasCovInv);
    J = computeJacobian(theta, obj);
    
    thetaCov = inv((J')*inv(measCov)*J);
    
    [~, numParams] = GetRobotCalibInfo();
    robotCov = thetaCov(1:(numParams - 3),1:(numParams - 3));
    
    y = max(svd(robotCov));
%     y = cond(J);
%     y = max(svd(thetaCov));
%     y = max(svd(xCov));
%     y = cond(thetaCov);
%     fprintf('\n%f\n', y);
end

function J = computeJacobian(theta, obj)
    res0 = obj(theta);
    J = nan(length(res0), length(theta));  %pre-allocate
    
    del = 1e-9*ones(length(theta), 1);
    del(end-24) = 1e-2;
    
    parfor iii = 1:length(theta)
      delTheta = theta;
      delTheta(iii) = delTheta(iii) + del(iii);
      J(:,iii) = (feval(obj, delTheta) - res0)/del(iii);
    end
end

function stop = plotBestFunctionsSa(~, optimvalues, flag, sampleRate, tSpan, T)

    AB = optimvalues.bestx;
    plotBestFunctions(1, AB, T, sampleRate, tSpan);
    
    stop = false;
end

function stop = plotBestFunctionsFMin(x, optimValues, state, sampleRate, tSpan, T)
    AB = x;
    plotBestFunctions(2, AB, T, sampleRate, tSpan);
    stop = false;

end


function stop = plotBestFunctionsPs(optimvalues, flag, T, sampleRate, tSpan)
    AB = optimvalues.x;
    plotBestFunctions(2, AB, T, sampleRate, tSpan);
    stop = false;
end

function plotBestFunctions(figNum, AB, T, sampleRate, tSpan)
    numMeas = sampleRate*(tSpan(2) - tSpan(1));
    t = linspace(tSpan(1), tSpan(2), numMeas);
    
    n = (size(AB, 1) - 1)/2;
    A = AB(1:n + 1,:);
    B = AB(n + 2:end,:);
    
    q = @(t) EvalVectorFourier(A, B, t, T);

    figure(figNum);
    plot(t, q(t));
    xlabel('time (sec)');
    ylabel('Joint Angle (rad)');
    title('Current Best Trajectory');
end

function [c, ceq] = nonlinearConstraint(AB, T)
    [~, c] = CheckJointLimits(AB, T);
    ceq = [];
end