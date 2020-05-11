function ComputeOptimalTrajectory()

robotName = 'Puma560';
ChangeRobot(robotName);

theta = GetNominalTheta();
jointLimits = GetJointLimits();

numWayPts = 13;
% numJoints = size(jointLimits, 1);

% numVars = numWayPts*numJoints;
jointLimitsLo = jointLimits(:,1);
jointLimitsHi = jointLimits(:,2);

LB = (jointLimitsLo*ones(1, numWayPts))';
UB = (jointLimitsHi*ones(1, numWayPts))';

% lb = reshape(LB, numVars, 1);
% ub = reshape(UB, numVars, 1);

sampleRate = 50;
tSpan = [0, 100];

alphCov = (0.01)^2*eye(3);
omegCov = (0.01)^2*eye(3);
zCov = blkdiag(alphCov, omegCov);

obj = @(qWayPts) computeObservabilityMeasure(qWayPts, sampleRate, tSpan, theta, zCov);

qWayPts0 = SampleJointSpace(numWayPts);

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.DisplayInterval = 1;
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf};
options.MaxTime = 10000;

qWayPts = simulannealbnd(obj, qWayPts0, LB, UB, options);

% options = optimoptions('fminunc');
% options.Display = 'iter';
% 
% qWayPts = fminunc(obj, qWayPts0, options);

save('OptimalTrajectory.mat', 'qWayPts');
end

function y = computeObservabilityMeasure(qWayPts, sampleRate, tSpan, theta, zCov)
    
    numPeriods = 11;
    T = (tSpan(2) - tSpan(1))/numPeriods;
    numMeas = sampleRate*T;
    qWayPts = repmat(qWayPts, numPeriods, 1);
    
    
    d = 5;
    [y, C] = FitVectorSpline(qWayPts, tSpan(1), tSpan(2), d);
    [yd, Cd, dd] = DerVectorSpline(y, C, d);
    [ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

    t = linspace(tSpan(1) + 5*T, tSpan(2) - 5*T, numMeas);

    q = @(t) EvalVectorSpline(y, C, d, t);
    qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
    qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);
    
    figure(1);
    plot(t, q(t));
    
    z = zeros(length(t), 6);
    zCovInv = inv(zCov);

    obj = @(theta) ComputeImuObjective(theta, t, q, qDot, qDDot, z, zCovInv);
    J = computeJacobian(theta, obj);

    y = cond(J);
end

function J = computeJacobian(theta, obj)
    res0 = obj(theta);
    J = nan(length(res0), length(theta));  %pre-allocate
    
    del = 1e-9;
    
    parfor iii = 1:length(theta)
      delTheta = theta;
      delTheta(iii) = delTheta(iii) + del;
      J(:,iii) = (feval(obj, delTheta) - res0)/del;
    end
end
