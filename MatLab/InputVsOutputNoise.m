clear;

aTruth = 1;
N = 20;
inputVar = 2;
outputVar = 2;

[xMeas, yMeas] = generateMeasurements(aTruth, N, inputVar, outputVar);

% Ordinary least-squares estimate
aLsq = yMeas \ xMeas;

% Total least-squares estimate
aTotLsq = lsqnonlin(@(a) getTotalResidual(a, xMeas, yMeas), aLsq);

% Plot results

h = figure(1);
clf;
hold on;
h.Color = [1,1,1];

scatter(xMeas, yMeas, 10, 'Filled', 'MarkerFaceColor', 'red');
fplot(@(x) aLsq*x);
fplot(@(x) aTotLsq*x);

axis([-2, 12, -2, 12]);

legend('Data', 'Ordinary Least Squares', 'Total Least Squares');


% Now do it a bunch of times and plot the results
numSims = 1000;
aLsq = zeros(1,numSims);
aTotLsq = zeros(1,numSims);

for iii = 1:numSims
    [xMeas, yMeas] = generateMeasurements(aTruth, N, inputVar, outputVar);
    aLsq(iii) = yMeas \ xMeas;
    aTotLsq(iii) = lsqnonlin(@(a) getTotalResidual(a, xMeas, yMeas), aLsq(iii));
end


% Plot results

h = figure(2);
clf;
hold on;
h.Color = [1,1,1];

histogram(aLsq);
histogram(aTotLsq);

legend('Ordinary Least Squares', 'Total Least Squares');

function res = getTotalResidual(a, xMeas, yMeas)
    points = [xMeas, yMeas]';
    
    numPoints = size(points, 2);
    
    % Project points onto line
    v = [1; a];
    v = v./norm(v);
    
    d = points - (ones(2,1)*(v')*points).*(v*ones(1,numPoints));
    res = d(:);
end


function [xMeas, yMeas] = generateMeasurements(aTruth, N, inputVar, outputVar)
    % Our model is of the form y = a*x.
    x = linspace(0, 10, N)';
    y = aTruth.*x;
    
    sig = [inputVar, 0; 0, outputVar];
    noise = mvnrnd([0;0], sig, N);

    xMeas = x + noise(:,1);
    yMeas = y + noise(:,2);
end