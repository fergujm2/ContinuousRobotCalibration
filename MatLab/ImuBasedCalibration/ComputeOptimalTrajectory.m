function ComputeOptimalTrajectory()

robotName = 'HebiX';
ChangeRobot(robotName);

theta = GetNominalTheta();
jointLimits = GetJointLimits();

alphCov = (0.1)^2*eye(3);
omegCov = (0.1)^2*eye(3);
zCov = blkdiag(alphCov, omegCov);

T = 30;
sampleRate = 1000;
tSpan = [0, 1*T];
n = 12;

obj = @(AB) computeObservabilityMeasure(AB, T, sampleRate, tSpan, theta, zCov);

numJoints = size(jointLimits, 1);
A0 = 0.1*(2.*rand(n + 1, numJoints) - 1);
B0 = 0.1*(2.*rand(n, numJoints) - 1);
AB0 = [A0; B0];

for iii = 1:100
    try
        obj(AB0);
        break
    catch
        AB0 = rand(size(AB0)) - 1;
        continue
    end
end

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.DisplayInterval = 1;
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf};
options.MaxTime = 10000;

LB = -0.4.*ones(size(AB0));
UB = 0.4.*ones(size(AB0));

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

save('OptimalTrajectory.mat', 'A', 'B');
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

%     figure(1);
%     plot(t, q(t));
    
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

function [c, ceq] = nonlinearConstraint(AB, T)
    % An AB in the feasible set is one s.t. c < 0. We want to compute the
    % maximum and minumum position velocity and accellerations for each
    % joint and make sure they're all within the correct bounds.
    
    n = (size(AB, 1) - 1)/2;
    A = AB(1:n + 1,:);
    B = AB(n + 2:end,:);
    
    [Ad, Bd] = DerVectorFourier(A, B, T);
    [Add, Bdd] = DerVectorFourier(Ad, Bd, T);
    
    t = linspace(0, T, 1000);
    
    q = EvalVectorFourier(A, B, t, T);
    qDot = EvalVectorFourier(Ad, Bd, t, T);
    qDDot = EvalVectorFourier(Add, Bdd, t, T);
    
    [qLimits, qDotLimits, qDDotLimits] = GetJointLimits();
    
    qLo = qLimits(:,1);
    qHi = qLimits(:,2);
    qDotLo = qDotLimits(:,1);
    qDotHi = qDotLimits(:,2);
    qDDotLo = qDDotLimits(:,1);
    qDDotHi = qDDotLimits(:,2);
    
    % Feasible means that c is negative
    c1 = qLo - min(q)';
    c2 = max(q)' - qHi;
    c3 = qDotLo - min(qDot)';
    c4 = max(qDot)' - qDotHi;
    c5 = qDDotLo - min(qDDot)';
    c6 = max(qDDot)' - qDDotHi;
    
    c = [c1; c2; c3; c4; c5; c6];
    ceq = 0;
end
