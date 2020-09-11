function ComputeOptimalTrajectoryGreedy()

T = 20;
sampleRate = 120;
knotsPerSecond = 1;
d = 3;
stepSpan = 5;

tSpan = [0, T];
k = length(tSpan(1):(1/knotsPerSecond):tSpan(2)) - 2;
y = makeExtendedKnots(tSpan(1), tSpan(end), k, d);

jointLimits = GetJointLimits();
jointMeans = mean(jointLimits,2);
n = d + k + 1;
C = repmat(jointMeans, 1, n);

% Initialize C with some random points
numInitPoints = 10;
initCols = C(:,1:numInitPoints) + 0.1*(rand(length(jointMeans), numInitPoints) - 0.5);
initColsInd = 1:numInitPoints;

C = insertColumnsIntoC(C, initColsInd, initCols);
[~, thetaCov] = objSpline(y, C, d, sampleRate, [0, 7], [0, 15]);

pointsPerStep = stepSpan*knotsPerSecond;
numPointsAdded = numInitPoints;

numSteps = floor((n - numInitPoints)/pointsPerStep);
maxSvdTheta = zeros(1, numSteps);

figure(1);
clf;

figure(2);
clf;

for iii = 1:numSteps
    newColsInd = (numPointsAdded + 1):(numPointsAdded + pointsPerStep);
    [C, thetaCov] = addNextSplinePoints(newColsInd, y, C, d, sampleRate, thetaCov);
    
    maxSvdTheta(iii) = max(svd(thetaCov));
    
    ts = tSpan(1):0.001:tSpan(end);
    q = EvalVectorSpline(y, C, d, ts);
    
    figure(1);
    plot(ts, q);
    
    figure(2);
    plot(1:iii, maxSvdTheta(1:iii), '-ok', 'MarkerFaceColor', 'red', 'LineWidth', 1);
    title(sprintf('maxSvdTheta: %.5f', maxSvdTheta(iii)));
    drawnow();
    
    numPointsAdded = numPointsAdded + pointsPerStep;
end

calibBools = GetRobotCalibInfo();

filename = sprintf('BSpline_d%.0f_step%.0f_%.0fs.mat', d, stepSpan, T);

fullFilename = fullfile('Output', filename);
save(fullFilename, 'y', 'C', 'sampleRate', 'thetaCov', 'calibBools');

end

function C = insertColumnsIntoC(C, newColsInd, newCols)
    C(:,newColsInd) = newCols;
end

function [CNew, thetaCov] = addNextSplinePoints(newColsInd, y, C, d, sampleRate, thetaCovOld)
    
    % Only consider the interval that newCols has an influence on.
    % Furthermore, do not consider the interval that the next step changes.
    tSpan = [y(newColsInd(1)), y(newColsInd(end) + 1)];
    
    % For joint limits, consider the full interval that newCols can change.
    tSpanJointLimits = [y(newColsInd(1)), y(newColsInd(end) + d + 1)];
    
    obj = @(newCols) objSpline(y, insertColumnsIntoC(C, newColsInd, newCols), d, sampleRate, tSpan, tSpanJointLimits, thetaCovOld);
    
    options = optimoptions('simulannealbnd');
    options.Display = 'iter';
    options.DisplayInterval = 1;
    options.MaxTime = 60*5;
    
    jointLimits = GetJointLimits();
    jointMeans = mean(jointLimits, 2);
    jointLengths = jointLimits(:,2) - jointLimits(:,1);
    
    LB = repmat(jointMeans - 0.25.*jointLengths, 1, length(newColsInd));
    UB = repmat(jointMeans + 0.25.*jointLengths, 1, length(newColsInd));
    
    newCols0 = repmat(jointMeans, 1, length(newColsInd));
    
    newCols = simulannealbnd(obj, newCols0, LB, UB, options);
    
    CNew = C;
    CNew(:,newColsInd) = newCols;
    
    % Now, get the real thetaCov for the full timeSpan so far.
    tSpanFull = [y(1), y(newColsInd(end) + 1)];
    [~, thetaCov] = objSpline(y, CNew, d, sampleRate, tSpanFull, tSpanJointLimits);
end

function y = makeExtendedKnots(a, b, k, d)
    numRep = d + 1;

    y0 = a*ones(1, numRep);
    yf = b*ones(1, numRep);

    yInt = linspace(a, b, k + 2);
    yInt = yInt(2:(end - 1));

    y = [y0, yInt, yf];
end

function [objVal, thetaCov] = objSpline(y, C, d, sampleRate, tSpan, tSpanJointLimits, thetaCovOld)
    
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

function stop = plotBestFunctionsSa(~, optimvalues, flag, sampleRate, tSpan, T)

    AB = optimvalues.bestx;
    plotBestFunctions(1, AB, T, sampleRate, tSpan);
    
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