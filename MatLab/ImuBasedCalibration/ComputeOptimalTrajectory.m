function ComputeOptimalTrajectory()

robotName = 'HebiX';
ChangeRobot(robotName);

theta = GetNominalTheta();
jointLimits = GetJointLimits();

numWayPts = 100;

jointLimitsLo = jointLimits(:,1);
jointLimitsHi = jointLimits(:,2);

LB = (jointLimitsLo*ones(1, numWayPts))';
UB = (jointLimitsHi*ones(1, numWayPts))';

sampleRate = 50;
tSpan = [0, 180];

alphCov = (0.1)^2*eye(3);
omegCov = (0.1)^2*eye(3);
zCov = blkdiag(alphCov, omegCov);

obj = @(qWayPts) computeObservabilityMeasure(qWayPts, sampleRate, tSpan, theta, zCov);

qWayPts0 = SampleJointSpace(numWayPts);

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.DisplayInterval = 1;
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf};
options.MaxTime = 1000;

qWayPts = simulannealbnd(obj, qWayPts0, LB, UB, options);

% options = optimoptions('fminunc');
% options.Display = 'iter';
% 
% qWayPts = fminunc(obj, qWayPts0, options);

save('OptimalTrajectory.mat', 'qWayPts');
end

function y = computeObservabilityMeasure(qWayPts, sampleRate, tSpan, theta, zCov)
    
    numMeas = sampleRate*(tSpan(2) - tSpan(1));
    t = linspace(tSpan(1) + 5, tSpan(2) - 5, numMeas);
    [q, qDot, qDDot] = FitJointValueFunctions(qWayPts, tSpan);

    figure(1);
    plot(t, q(t));
    
    z = zeros(length(t), 6);
    zCovInv = inv(zCov);

    obj = @(theta) ComputeImuObjective(theta, t, q, qDot, qDDot, z, zCovInv);
    J = computeJacobian(theta, obj);
    
    zDataCov = diag(zCov)*ones(1, size(z, 1));
    measCov = zDataCov(:);
    measCov = spdiags(measCov, 0, length(measCov), length(measCov));
    thetaCov = inv((J')*inv(measCov)*J);
    
%     y = cond(J);
    y = max(svd(thetaCov));
%     y = cond(thetaCov);
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
