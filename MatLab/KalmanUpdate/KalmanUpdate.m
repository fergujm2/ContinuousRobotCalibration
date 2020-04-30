function [xHatNew, PHatNew] = KalmanUpdate(xHat, PHat, u, y, F, G, H, V, W)
% Given current state estimate information, system inputs, sensor readings,
% system model, and probablistic model, estimates the new state information
% based on the linear Kalman equations (see Ch. 8 of Principles of Robot 
% Motion by Choset et al.)

% Input
%   xHat    | n x 1 | current state estimate
%   PHat    | n x n | current state covariance matrix estimate
%   u       | m x 1 | current system input
%   y       | p x 1 | current sensor reading
%   F       | n x n | linear system dynamics
%   G       | n x m | linear input dynamics
%   H       | p x n | linear output 
%   V       | n x n | current process noise covariance
%   W       | p x p | current sensor noise covariance

% Output 
%   xHatNew | n x 1 | new state estimate after update
%   PHatNew | n x n | new state covariance matrix estimate


% Predict
xPredict = F*xHat + G*u;
PPredict = F*PHat*(F') + V;

% Update
nu = y - H*xPredict;
S = H*PPredict*(H') + W;
R = PPredict*(H')/(S);

xHatNew = xPredict + R*nu;
PHatNew = PPredict - R*H*PPredict;
end