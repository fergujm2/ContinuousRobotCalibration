function ComputeOptimalTrajectory()

T = 5*60;

sampleRate = 120;
knotsPerSecond = 1;
d = 3;
stepSpan = 5;

tSpan = [0, T];
k = length(tSpan(1):(1/knotsPerSecond):tSpan(2)) - 2;
y = MakeExtendedKnots(tSpan(1), tSpan(end), k, d);

jointLimits = GetJointLimits();
jointMeans = mean(jointLimits,2);
n = d + k + 1;
C = repmat(jointMeans, 1, n);

% Initialize C with some random points
numZeroPoints = 3;

[~, thetaCovInit] = GetThetaNominal();

pointsPerStep = stepSpan*knotsPerSecond;
numPointsAdded = numZeroPoints;

numSteps = floor((n - numZeroPoints)/pointsPerStep);
maxSvdTheta = zeros(1, numSteps + 1);
maxSvdTheta(1) = max(svd(thetaCovInit));
thetaCov = zeros([size(thetaCovInit), numSteps + 1]);
thetaCov(:,:,1) = thetaCovInit;

figure(1);
clf;

figure(2);
clf;

for iii = 1:numSteps
    newColsInd = (numPointsAdded + 1):(numPointsAdded + pointsPerStep);
    [C, thetaCov(:,:,(iii + 1))] = addNextSplinePoints(newColsInd, y, C, d, sampleRate, thetaCov(:,:,iii), thetaCovInit);
    maxSvdTheta(iii + 1) = max(svd(thetaCov(:,:,(iii + 1))));
    
    ts = tSpan(1):0.001:tSpan(end);
    q = EvalVectorSpline(y, C, d, ts);
    
    figure(1);
    plot(ts, q);
    
    figure(2);
    plot((2:(iii + 1)) - 1, maxSvdTheta(2:(iii + 1)), '-ok', 'MarkerFaceColor', 'red', 'LineWidth', 1);
    title(sprintf('maxSvdTheta: %.5f', maxSvdTheta(iii + 1)));
    drawnow();
    
    numPointsAdded = numPointsAdded + pointsPerStep;
end

calibBools = GetRobotCalibInfo();
[qLimits, qDotLimits, qDDotLimits] = GetJointLimits();

filename = sprintf('BSpline_d%.0f_step%.0f_%.0fs.mat', d, stepSpan, T);
fullFilename = fullfile('Output', filename);

save(fullFilename, 'y', 'C', 'd', 'k', 'n', 'knotsPerSecond', 'tSpan', 'sampleRate', 'thetaCov', 'maxSvdTheta', 'calibBools', 'numSteps', 'stepSpan', 'pointsPerStep', 'qLimits', 'qDotLimits', 'qDDotLimits');

end

function C = insertColumnsIntoC(C, newColsInd, newCols)
    C(:,newColsInd) = reshape(newCols, 6, []);
end

function [CNew, thetaCov] = addNextSplinePoints(newColsInd, y, C, d, sampleRate, thetaCovOld, thetaCovInit)
    
    % Only consider the interval that newCols has an influence on.
    % Furthermore, do not consider the interval that the next step changes.
    tSpan = [y(newColsInd(1)), y(newColsInd(end) + 1)];
    
    % For joint limits, consider the full interval that newCols can change.
    tSpanJointLimits = [y(newColsInd(1)), y(newColsInd(end) + d + 1)];
    
    % Obj takes newCols as a vector
    obj = @(newCols) ComputeObservability(y, insertColumnsIntoC(C, newColsInd, newCols), d, sampleRate, tSpan, tSpanJointLimits, thetaCovOld);
    con = @(newCols) nonlinearConstraint(y, insertColumnsIntoC(C, newColsInd, newCols), d, tSpanJointLimits);
    objconstr = @(newCols) objConstr(newCols, obj, con);
    
    jointLimits = GetJointLimits();
    jointMeans = mean(jointLimits, 2);
    jointLengths = jointLimits(:,2) - jointLimits(:,1);
    
%     newCols0 = repmat(jointMeans, 1, length(newColsInd));
    LB = repmat(jointMeans - 0.25.*jointLengths, 1, length(newColsInd));
    UB = repmat(jointMeans + 0.25.*jointLengths, 1, length(newColsInd));
    lb = LB(:);
    ub = UB(:);
    
    options = optimoptions('surrogateopt');
    options.Display = 'iter';
    options.PlotFcn = {@optimplotx,@optimplotconstrviolation,@surrogateoptplot};
    options.UseParallel = true;
    options.MaxFunctionEvaluations = 1e6;
    options.OutputFcn = @outFun;
    
    newCols = surrogateopt(objconstr, lb, ub, options);
    
    options = optimoptions('fmincon');
    options.Algorithm = 'active-set';
    options.Display = 'iter';
    options.PlotFcn = {@optimplotx,@optimplotfval,@optimplotconstrviolation};
    options.UseParallel = true;
    options.MaxIterations = 150;
    options.StepTolerance = 1e-5;
    options.RelLineSrchBnd = 0.01;
    options.RelLineSrchBndDuration = options.MaxIterations;
    
    newCols = fmincon(obj, newCols, [], [], [], [], [], [], con, options);
    
    newCols = reshape(newCols, 6, []);
    
    CNew = C;
    CNew(:,newColsInd) = newCols;
    
    % Now, get the real thetaCov for the full timeSpan so far.
    tSpanFull = [y(1), y(newColsInd(end) + 1)];
    [~, thetaCov] = ComputeObservability(y, CNew, d, sampleRate, tSpanFull, tSpanJointLimits, thetaCovInit);
end

function [c, ceq] = nonlinearConstraint(y, C, d, tSpan)
    [yd, Cd, dd] = DerVectorSpline(y, C, d);
    [ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);
    
    q = @(t) EvalVectorSpline(y, C, d, t);
    qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
    qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);
    
    [~, c] = CheckJointLimits(q, qDot, qDDot, tSpan);
    
    ceq = [];
end

function S = objConstr(x, obj, con)
    S.Fval = obj(x);
    S.Ineq = con(x);
end

function stop = outFun(x, optimValues, state)
    stop = and(optimValues.constrviolation <= 1e-3, optimValues.funccount > 2500);
end