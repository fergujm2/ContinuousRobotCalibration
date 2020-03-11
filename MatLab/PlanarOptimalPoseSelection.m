clear;
close all;

m = 10;
n = 2;

numVars = m*n;

lb = 0*pi/180;
ub = 360*pi/180;

LB = lb.*ones(numVars, 1);
UB = ub.*ones(numVars, 1);

L1 = 1;
L2 = 1;
piNominal = [0, 0, 0, L1, 0, 0, L2, 0, 0];

x0 = 2*pi*rand(numVars, 1);
obj = @(x) computeObjective(x, piNominal, false);

options = optimoptions('simulannealbnd');
options.PlotFcns = {@saplotbestx,@saplotbestf,@saplotx,@saplotf};
options.MaxTime = 10;

xStar = simulannealbnd(obj, x0, LB, UB, options);

fStar = computeObjective(xStar, piNominal, true);


function f = computeObjective(x, piNominal, showPlot)
    if showPlot
        figure(1);
        clf;
        hold on;
    end
    
    m = length(x)/2;
    q = reshape(x, m, 2);
    
    J = computeIdJacobian(q, piNominal, showPlot);

    S = svd(J);
    L = size(J,2);
    f = -((prod(S))^(1/L))/sqrt(m);

    if showPlot
        daspect([1,1,1]);
        axis([-5, 5, -5, 5]);
        drawnow();
    end
end

function J = computeIdJacobian(q, piNominal, showPlot)
    numConfigs = size(q, 1);
    
    J = zeros(3*numConfigs, 7);
    
    for iii = 1:numConfigs
        Ji = computeExtendedJacobian(q(iii,:), piNominal, showPlot);
        
        indLo = 3*iii - 2;
        indHi = indLo + 2;
        
        J(indLo:indHi,:) = Ji;
    end
end

function J = computeExtendedJacobian(q, piNominal, showPlot)
    theta1 = q(1);
    theta2 = q(2);
    
    L1 = piNominal(4);
    L2 = piNominal(7);
    
    J = [1, 0, - L2*sin(theta1 + theta2) - L1*sin(theta1), cos(theta1), -L2*sin(theta1 + theta2), cos(theta1 + theta2), 0;
         0, 1,   L2*cos(theta1 + theta2) + L1*cos(theta1), sin(theta1),  L2*cos(theta1 + theta2), sin(theta1 + theta2), 0;
         0, 0,                                          1,           0,                        1,                    0, 1];
    
    if showPlot
        computeForwardKinematics(q, piNominal, showPlot);
    end
end

function x = computeForwardKinematics(q, piNominal, showPlot)
    theta1 = q(1);
    theta2 = q(2);
    
    a0 = piNominal(1);
    b0 = piNominal(2);
    gamma0 = piNominal(3);
    a1 = piNominal(4);
    b1 = piNominal(5);
    gamma1 = piNominal(6);
    a2 = piNominal(7);
    b2 = piNominal(8);
    gamma2 = piNominal(9);
    
    x0 = [a0 + a1*cos(gamma0 + theta1) - b1*sin(gamma0 + theta1);
          b0 + b1*cos(gamma0 + theta1) + a1*sin(gamma0 + theta1);
          gamma0 + gamma1 + theta1];
      
    x = x0 + [a2*cos(gamma0 + gamma1 + theta1 + theta2) - b2*sin(gamma0 + gamma1 + theta1 + theta2);
              b2*cos(gamma0 + gamma1 + theta1 + theta2) + a2*sin(gamma0 + gamma1 + theta1 + theta2);
              gamma2 + theta2];
    
    if showPlot
        X = [[0;0], x0(1:2), x(1:2)];
        plot(X(1,:), X(2,:), '-O', 'MarkerSize', 12, 'LineWidth', 1)
    end
end