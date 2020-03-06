clear;
close all;

robot = GetRobot();

dhParams = GetDhParameters(robot);

calibBools = false(size(dhParams));

calibBools(1,2) = true;
calibBools(3,1) = true;
calibBools(5,3) = true;

numParams = sum(sum(calibBools));

m = 4;
n = length(robot.homeConfiguration);

numVars = m*n;

jointLimits = zeros(n, 2);

for iii = 1:n
    jointLimits(iii,:) = robot.Bodies{1}.Joint.PositionLimits;
end

jointLimitsLo = jointLimits(:,1);
jointLimitsHi = jointLimits(:,2);

LB = (jointLimitsLo*ones(1, m))';
UB = (jointLimitsHi*ones(1, m))';

lb = reshape(LB, numVars, 1);
ub = reshape(UB, numVars, 1);

x0 = unifrnd(lb, ub);
obj = @(x) computeObjective(x, robot, dhParams, calibBools, false);

% xStar = fmincon(obj, x0, [], [], [], [], LB, UB);

options = optimoptions('simulannealbnd');
options.Display = 'iter';
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf};
options.MaxTime = 1000;
% options.AnnealingFcn = 'annealingfast';
% options.InitialTemperature = 1000;
xStar = simulannealbnd(obj, x0, LB, UB, options);

% options = optimoptions('ga');
% options.PlotFcn = @gaplotbestf;
% options.Display = 'iter';
% options.HybridFcn = 'patternsearch';
% options.PopulationSize = 20;
% options.UseParallel = true;

% xStar = ga(obj, numVars, [], [], [], [], lb, ub, [], options);

% Plot the results
fStar = computeObjective(xStar, robot, dhParams, calibBools, true);


function f = computeObjective(x, robot, dhParams, calibBools, showPlot)
    if showPlot
        figure(1);
        clf;
        hold on;
    end
    
    n = length(robot.homeConfiguration);
    m = length(x)/n;
    q = reshape(x, m, n);
    
    J = computeIdJacobian(q, robot, dhParams, calibBools, showPlot);
    f = cond(J);
    
    if showPlot
        daspect([1,1,1]);
        axis([-1, 1, -1, 1, -1, 1]);
        view([30, 30]);
        grid on;
        drawnow();
    end
end

function J = computeIdJacobian(q, robot, dhParams, calibBools, showPlot)
    numConfigs = size(q, 1);
    numParams = length(dhParams(calibBools));
    
    J = zeros(3*numConfigs, numParams);
    
    for iii = 1:numConfigs
        Ji = computeExtendedJacobian(q(iii,:), robot, dhParams, calibBools, showPlot);
        
        indLo = 3*iii - 2;
        indHi = indLo + 2;
        
        J(indLo:indHi,:) = Ji;
    end
end

function J = computeExtendedJacobian(q, robot, dhParams, calibBools, showPlot)
    numParams = length(dhParams(calibBools));
    x = zeros(numParams, 1);
    
    J = zeros(3, numParams);
    
    del = 1e-10;
    
    for iii = 1:numParams
        x1 = x;
        x2 = x;
        
        x1(iii) = x1(iii) - del;
        x2(iii) = x2(iii) + del;
        
        x1 = ComputeForwardKinematics(robot, q, x1, dhParams, calibBools);
        x2 = ComputeForwardKinematics(robot, q, x2, dhParams, calibBools);
        
        J(:,iii) = (x2 - x1)./(2*del);
    end
    
    if showPlot
        qConfig = robot.homeConfiguration;
        for iii = 1:length(q)
            qConfig(iii).JointPosition = q(iii);
        end
        robot.show(qConfig);
    end
end
