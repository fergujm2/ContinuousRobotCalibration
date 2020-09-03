function ComputeOptimalTrajectory()

jointLimits = GetJointLimits();
numJoints = size(jointLimits, 1);

T = 10;
sampleRate = 120;
tSpan = [0, 1*T];
n = 5;

obj = @(AB) computeObservabilityMeasure(AB, T, sampleRate, tSpan);
AB0 = GetRandomAB(n, numJoints, T);

for iii = 1:100
    try
        fprintf('Random try number: %d\n', iii);
        y0 = obj(AB0);
        break
    catch
        AB0 = GetRandomAB(n, numJoints, T);
        continue
    end
end

fprintf('Starting SA with y0 = %f', y0);

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.DisplayInterval = 1;

customPlotFcn = @(x, optimvalues, flag) plotBestFunctions(x, optimvalues, flag, sampleRate, tSpan, T);
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf,customPlotFcn};
% options.PlotInterval = 10;
options.MaxTime = 50000;
options.AnnealingFcn = @(optimValues,problem) AnnealingJointLimits(optimValues,problem,T);
options.InitialTemperature = 50;

bound = 2.*ones(size(AB0));
LB = -bound;
UB = bound;

AB = simulannealbnd(obj, AB0, LB, UB, options);

[~, thetaCov] = obj(AB);

A = AB(1:(n + 1),:);
B = AB((n + 2):end,:);

filename = fullfile('Output', 'OptimalTrajectory.mat');
save(filename, 'A', 'B', 'T', 'sampleRate', 'thetaCov');
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
    
%     y = cond(J);
    y = max(svd(thetaCov));
%     y = max(svd(xCov));
%     y = cond(thetaCov);
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

function stop = plotBestFunctions(~, optimvalues, flag, sampleRate, tSpan, T)

    numMeas = sampleRate*(tSpan(2) - tSpan(1));
    t = linspace(tSpan(1), tSpan(2), numMeas);
    
    AB = optimvalues.bestx;
    n = (size(AB, 1) - 1)/2;
    A = AB(1:n + 1,:);
    B = AB(n + 2:end,:);
    
    q = @(t) EvalVectorFourier(A, B, t, T);

    figure(1);
    plot(t, q(t));
    xlabel('time (sec)');
    ylabel('Joint Angle (rad)');
    title('Current Best Trajectory');
    
    stop = false;
end