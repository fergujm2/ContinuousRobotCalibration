clear;
close all;

calibBools = logical([1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   1   0   1   1   1   0   0   0   0   1   1   1   0   0   0]);
numParamsTotal = length(calibBools);
numParams = sum(calibBools);

jointLimits = [-pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4
               -pi/4, pi/4];

m = 30;
n = size(jointLimits, 1);

numVars = m*n;

jointLimitsLo = jointLimits(:,1);
jointLimitsHi = jointLimits(:,2);

LB = (jointLimitsLo*ones(1, m))';
UB = (jointLimitsHi*ones(1, m))';

lb = reshape(LB, numVars, 1);
ub = reshape(UB, numVars, 1);

y0 = mean([lb, ub], 2);
obj = @(y) computeObjective(y, calibBools, false);

% options = optimoptions('fmincon');
% options.Display = 'iter';
% yStar = fmincon(obj, y0, [], [], [], [], lb, ub, [], options);

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.DisplayInterval = 10000000;
% options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf};
% options.MaxTime = 1000;
yStar = simulannealbnd(obj, y0, lb, ub, options);

% options = optimoptions('ga');
% % options.PlotFcn = @gaplotbestf;
% options.Display = 'iter';
% options.PopulationSize = 100;
% options.UseParallel = true;
% yStar = ga(obj, numVars, [], [], [], [], lb, ub, [], options);

% Plot the results
fStar = computeObjective(yStar, calibBools, true);
qStar = reshape(yStar, m, n);

filename = fullfile('Output', 'OptimalPoses', [num2str(m), '.mat']);
save(filename, 'qStar', 'fStar');

function f = computeObjective(y, calibBools, showPlot)
    
    n = 6;
    m = length(y)/n;
    q = reshape(y, m, n);
    J = ComputeIdJacobian(q, calibBools, false);
    
    S = svd(J);
    L = size(J,2);
    f = -((prod(S))^(1/L))/sqrt(m);
    
    if showPlot
        figure(1);
        clf;
        hold on;
        
        e = zeros(length(calibBools), 1);
        
        for iii = 1:m
            ComputeForwardKinematics(q(iii,:), e, true);
        end
        
        daspect([1,1,1]);
        axis([-1, 1, -1, 1, -1, 1]);
        view([30, 30]);
        grid on;
        drawnow();
    end
end