function [objVal, thetaCov] = ComputeObservability(y, C, d, sampleRate, tSpan, tSpanJointLimits, thetaCovOld)
    
    % Turn spline coefficients into anonymous functions
    [yd, Cd, dd] = DerVectorSpline(y, C, d);
    [ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);
    
    q = @(t) EvalVectorSpline(y, C, d, t);
    qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
    qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);
    
    % Return very large value if the trajectory is outside limits
    if ~CheckJointLimits(q, qDot, qDDot, tSpanJointLimits)
        objVal = 1e16;
        thetaCov = inf;
        return
    end
    
    thetaCov = computeThetaCov(q, qDot, qDDot, sampleRate, tSpan);
    
    % If we have a prior covariance for theta, compute the posterior
    if nargin == 7
        thetaCov = inv(inv(thetaCovOld) + inv(thetaCov));
    end
    
%     calibBools = GetRobotCalibInfo();
%     numRobotParams = sum(calibBools) - 3;
%     
%     robotParamCov = thetaCov(1:numRobotParams,1:numRobotParams);
%     
%     objVal = max(svd(robotParamCov));
    objVal = max(svd(thetaCov));
end

function thetaCov = computeThetaCov(q, qDot, qDDot, sampleRate, tSpan)
    theta = GetThetaNominal();
    
    numMeas = sampleRate*(tSpan(2) - tSpan(1));
    t = linspace(tSpan(1), tSpan(2), numMeas);
    
    z = zeros(length(t), 6);
    
    measCov = GetMeasurementCovariance(numMeas);
    
    % Now we need the covariance to be identity to make sure we're using
    % the correct Jacobian.
    sqrtMeasCovInv = measCov \ measCov;
    
    obj = @(theta) ComputeImuObjective(theta, q, qDot, qDDot, t, z, sqrtMeasCovInv);
    J = computeJacobian(theta, obj);
    
    thetaCov = inv((J')*inv(measCov)*J);
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