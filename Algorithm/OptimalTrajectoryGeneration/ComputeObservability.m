function [objVal, thetaCov] = ComputeObservability(y, C, d, sampleRate, tSpan, tSpanJointLimits, thetaCovOld, jointLimitTol)
    
    % Turn spline coefficients into anonymous functions
    [yd, Cd, dd] = DerVectorSpline(y, C, d);
    [ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);
    
    q = @(t) EvalVectorSpline(y, C, d, t);
    qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
    qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);
    
    if nargin < 8
        jointLimitTol = 0;
    end
    
    % Return very large value if the trajectory is outside limits
    if ~isempty(tSpanJointLimits)
        if ~CheckJointLimits(q, qDot, qDDot, tSpanJointLimits, jointLimitTol)
            objVal = 1e16;
            thetaCov = inf;
            return
        end
    end
    
    thetaCov = computeThetaCov(q, qDot, qDDot, sampleRate, tSpan);
    
    % If we have a prior covariance for theta, compute the posterior
    if thetaCovOld ~= inf
        thetaCov = inv(inv(thetaCovOld) + inv(thetaCov));
    end

    objVal = max(svd(thetaCov));
end

function thetaCov = computeThetaCov(q, qDot, qDDot, sampleRate, tSpan)
    theta = GetThetaNominal();
    
    numMeas = sampleRate*(tSpan(2) - tSpan(1));
    
    t = linspace(tSpan(1), tSpan(2), numMeas);
    z = zeros(length(t), 6);
    
%     theta = GetThetaNominal();
%     zCov = ComputeZCovPrior(theta, q(t), qDot(t));

%     TBin = 0.2;
%     zCov = ComputeZCovPost(tImu, z, TBin);

%     TBin = tImu(end) - tImu(1);
%     zCov = ComputeZCovPost(tImu, z, TBin);
    
    % Assume constant covariance taken from 10 minutes of the 20200922
    % Calibration data set
    zCov = GetCovariances();
    zCov = repmat(zCov, numMeas, 1);
    
    measCov = reshape(zCov', [], 1);
    measCovInv = 1./measCov;

    % Now we need the covariance to be identity to make sure we're using
    % the correct Jacobian.
    sqrtMeasCovInv = ones(size(measCov));
    
    obj = @(theta) ComputeImuObjective(theta, q, qDot, qDDot, t, z, sqrtMeasCovInv);
    J = computeJacobian(theta, obj);
    
    thetaCov = inv((J')*(measCovInv*ones(1,size(J,2)).*J));
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