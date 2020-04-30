function [xHatNew, PHatNew] = EkfUpdate(xHat, PHat, u, y, t, f, h, V, W)
% Given current state estimate information, system inputs, sensor readings,
% system model, and probablistic model, estimates the new state information
% based on the Extended Kalman Filter (see Ch. 8 of Principles of Robot 
% Motion by Choset et al.)

% Input
%   xHat       | n x 1           | current state estimate
%   PHat       | n x n           | current state covariance matrix estimate
%   u          | m x 1           | current system input
%   y          | p x 1           | current sensor reading
%   t          | 1 x 1           | current time, i.e. t_i
%   f(x, u, t) | function_handle | nonlinear system dynamics
%   h(x, t)    | function_handle | nonlinear system output 
%   V          | n x n           | current process noise covariance
%   W          | p x p           | current sensor noise covariance

% Output
%   xHatNew    | n x 1           | new state estimate after update
%   PHatNew    | n x n           | new state covariance matrix estimate


% Predict
xPredict = f(xHat, u, t);
F = computeJacobian(@(x) f(x, u, t), xHat);
PPredict = F*PHat*(F') + V;

% Update
nu = y - h(xPredict, t);
H = computeJacobian(@(x) h(x, t), xPredict);
S = H*PPredict*(H') + W;
R = PPredict*(H') / S;
    
xHatNew = xPredict + R*nu;
PHatNew = PPredict - R*H*PPredict;
end

function J = computeJacobian(f, x)
    del = 1e-10; % Finite difference
    
    % Evaluate at center point for finite differences
    f0 = f(x);
    
    J = zeros(length(f0), length(x));
    
    % Fill in each column of the numerical jacobian
    for iii = 1:length(x)
        % Create the right and left points for the finite differences
        x1 = x;
        x2 = x;
        
        x1(iii) = x1(iii) - del;
        x2(iii) = x2(iii) + del;
        
        % Evaluate at the right point
        delf = f(x2) - f(x1);
        
        % Compute the local 'slope'
        J(:,iii) = delf./(2*del);
    end
end