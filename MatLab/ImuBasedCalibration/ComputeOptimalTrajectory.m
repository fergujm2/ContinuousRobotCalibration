function ComputeOptimalTrajectory()

robotName = 'AuboI5';
ChangeRobot(robotName);

theta = GetNominalTheta();
jointLimits = GetJointLimits();

alphCov = (1e0)^2*eye(3);
omegCov = (1e-1)^2*eye(3);
zCov = blkdiag(alphCov, omegCov);

T = 10;
sampleRate = 200;
tSpan = [0, 1*T];
n = 5;

numJoints = size(jointLimits, 1);

obj = @(AB) computeObservabilityMeasure(AB, T, sampleRate, tSpan, theta, zCov);
AB0 = GetRandomAB(n, numJoints);

for iii = 1:100
    try
        fprintf('Random try number: %d\n', iii);
        y0 = obj(AB0);
        break
    catch
        AB0 = GetRandomAB(n, numJoints);
        continue
    end
end

fprintf('Starting SA with y0 = %f', y0);

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.DisplayInterval = 1;

customPlotFcn = @(x, optimvalues, flag) plotBestFunctions(x, optimvalues, flag, sampleRate, tSpan, T);
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf,customPlotFcn};
options.PlotInterval = 10;
options.MaxTime = 1000;
options.AnnealingFcn = @(optimValues,problem) AnnealingJointLimits(optimValues,problem,T);
options.InitialTemperature = 50;

% temp = ((linspace(T, 1, n)/T)'*ones(1, 6)).^1;
% bound = 2.*[ones(1, numJoints); temp; temp];

bound = 2.*ones(size(AB0));
LB = -bound;
UB = bound;

AB = simulannealbnd(obj, AB0, LB, UB, options);

% options = optimoptions('fmincon');
% options.Display = 'iter';
% options.InitTrustRegionRadius = 0.1;
% options.UseParallel = true;
% 
% con = @(AB) nonlinearConstraint(AB, T);
% 
% AB = fmincon(obj, AB0, [], [], [], [], [], [], con, options);

A = AB(1:(n + 1),:);
B = AB((n + 2):end,:);

save('OptimalTrajectoryOffset.mat', 'A', 'B');
end

function y = computeObservabilityMeasure(AB, T, sampleRate, tSpan, theta, zCov)
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
    zCovInv = inv(zCov);

    obj = @(theta) ComputeImuObjective(theta, t, q, qDot, qDDot, z, zCovInv);
    J = computeJacobian(theta, obj);
    
    zDataCov = diag(zCov)*ones(1, size(z, 1));
    measCov = zDataCov(:);
    measCov = spdiags(measCov, 0, length(measCov), length(measCov));
    thetaCov = inv((J')*inv(measCov)*J);
    
    [~, numParams] = GetRobotCalibInfo();
    xCov = thetaCov(1:numParams,1:numParams);
    
%     y = cond(J);
%     y = max(svd(thetaCov));
    y = max(svd(xCov));
%     y = cond(thetaCov);
end

function J = computeJacobian(theta, obj)
    res0 = obj(theta);
    J = nan(length(res0), length(theta));  %pre-allocate
    
    del = 1e-6;
    
    parfor iii = 1:length(theta)
      delTheta = theta;
      delTheta(iii) = delTheta(iii) + del;
      J(:,iii) = (feval(obj, delTheta) - res0)/del;
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