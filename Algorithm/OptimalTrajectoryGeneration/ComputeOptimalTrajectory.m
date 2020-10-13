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
numInitPoints = 7;
numZeroPoints = 3;
initPointsNoise =  0.1*(rand(length(jointMeans), numInitPoints - numZeroPoints) - 0.5);
initCols = C(:,1:numInitPoints) + [zeros(length(jointMeans), numZeroPoints), initPointsNoise];
initColsInd = 1:numInitPoints;

C = insertColumnsIntoC(C, initColsInd, initCols);
[~, thetaCovInit] = ComputeObservability(y, C, d, sampleRate, [0, 25], [0, 15], inf);

pointsPerStep = stepSpan*knotsPerSecond;
numPointsAdded = numInitPoints;

numSteps = floor((n - numInitPoints)/pointsPerStep);
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
    [C, thetaCov(:,:,(iii + 1))] = addNextSplinePoints(newColsInd, y, C, d, sampleRate, thetaCov(:,:,iii));
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
    C(:,newColsInd) = newCols;
end

function [CNew, thetaCov] = addNextSplinePoints(newColsInd, y, C, d, sampleRate, thetaCovOld)
    
    % Only consider the interval that newCols has an influence on.
    % Furthermore, do not consider the interval that the next step changes.
    tSpan = [y(newColsInd(1)), y(newColsInd(end) + 1)];
    
    % For joint limits, consider the full interval that newCols can change.
    tSpanJointLimits = [y(newColsInd(1)), y(newColsInd(end) + d + 1)];
    
    obj = @(newCols) ComputeObservability(y, insertColumnsIntoC(C, newColsInd, newCols), d, sampleRate, tSpan, tSpanJointLimits, thetaCovOld);
    
    % Compute initial set with simulated annealing
    options = optimoptions('simulannealbnd');
    options.Display = 'iter';
    options.DisplayInterval = 1;
    options.PlotFcns = {@saplotbestf,@saplotbestx,@saplotf};
    options.PlotInterval = 10;
    options.MaxIterations = 2000; % 2000
    
    jointLimits = GetJointLimits();
    jointMeans = mean(jointLimits, 2);
    jointLengths = jointLimits(:,2) - jointLimits(:,1);
    
    LB = repmat(jointMeans - 0.25.*jointLengths, 1, length(newColsInd));
    UB = repmat(jointMeans + 0.25.*jointLengths, 1, length(newColsInd));
    
    newCols0 = repmat(jointMeans, 1, length(newColsInd));
    newCols = simulannealbnd(obj, newCols0, LB, UB, options);
    
    % Refine new set of points with pattern search
    options = optimoptions('patternsearch');
    options.Display = 'iter';
    options.MaxIterations = 200; % 200
    options.InitialMeshSize = 0.5;
    options.MaxMeshSize = 0.5;
    options.UseParallel = true;
    options.PollMethod = 'GPSPositiveBasisNp1';
    options.UseCompletePoll = true;
    options.Cache = 'on';
    options.PlotFcn = {@psplotbestf, @psplotbestx, @psplotmeshsize};
    
    newCols = patternsearch(obj, newCols, [], [], [], [], [], [], [], options);

    CNew = C;
    CNew(:,newColsInd) = newCols;
    
    % Now, get the real thetaCov for the full timeSpan so far.
    tSpanFull = [y(1), y(newColsInd(end) + 1)];
    [~, thetaCov] = ComputeObservability(y, CNew, d, sampleRate, tSpanFull, tSpanJointLimits, inf);
end