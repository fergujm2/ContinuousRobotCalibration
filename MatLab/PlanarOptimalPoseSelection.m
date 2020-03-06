clear;
close all;

m = 30;
n = 2;

numVars = m*n;

lb = 45*pi/180;
ub = 135*pi/180;

LB = lb.*ones(numVars, 1);
UB = ub.*ones(numVars, 1);

p = [3, 0, 2, 0];

x0 = 1.5.*(2*rand(numVars, 1) - 1) + pi/2;
obj = @(x) computeObjective(x, p, false);

% xStar = fmincon(obj, x0, [], [], [], [], LB, UB);

% options = optimoptions('simulannealbnd');
% options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf};
% options.MaxTime = 100;
% options.AnnealingFcn = 'annealingfast';
% options.InitialTemperature = 1000;
% xStar = simulannealbnd(obj, x0, LB, UB, options);

options = optimoptions('ga');
% options.PlotFcn = @gaplotbestf;
options.Display = 'iter';
% options.HybridFcn = 'patternsearch';
options.PopulationSize = 2000;
options.UseParallel = true;

xStar = ga(obj, numVars, [], [], [], [], LB, UB, [], options);

% Plot the results
fStar = computeObjective(xStar, p, true);


function f = computeObjective(x, p, showPlot)
    if showPlot
        figure(1);
        clf;
        hold on;
    end
    
    m = length(x)/2;
    q = reshape(x, m, 2);
    J = computeIdJacobian(q, p, showPlot);
    f = cond(J);
    
    if showPlot
        daspect([1,1,1]);
        axis([-5, 5, -5, 5]);
        drawnow();
    end
end

function J = computeIdJacobian(q, p, showPlot)
    numConfigs = size(q, 1);
    numParams = length(p);
    
    J = zeros(2*numConfigs, numParams);
    
    for iii = 1:numConfigs
        Ji = computeExtendedJacobian(q(iii,:), p, showPlot);
        
        indLo = 2*iii - 1;
        indHi = indLo + 1;
        
        J(indLo:indHi,:) = Ji;
    end
end

function J = computeExtendedJacobian(q, p, showPlot)
    numParams = length(p);
    
    J = zeros(2, numParams);
    
    del = 1e-10;
    
    for iii = 1:numParams
        p1 = p;
        p2 = p;
        
        p1(iii) = p1(iii) - del;
        p2(iii) = p2(iii) + del;
        
        x1 = computeForwardKinematics(q, p1, showPlot);
        x2 = computeForwardKinematics(q, p2, false);
        
        J(:,iii) = (x2 - x1)./(2*del);
    end
end

function x = computeForwardKinematics(q, p, showPlot)
    l1 = p(1);
    theta1 = p(2);
    l2 = p(3);
    theta2 = p(4);
    
    x0 = l1.*[cos(q(1) + theta1); sin(q(1) + theta1)];
    x = x0 + l2.*[cos(q(2) + theta2); sin(q(2) + theta2)];
    
    if showPlot
        X = [[0;0], x0, x];
        plot(X(1,:), X(2,:), '-O', 'MarkerSize', 12, 'LineWidth', 1)
    end
end