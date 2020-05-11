function [theta, thetaCov] = PackTheta(x, g, tau, s, xCov, gCov, tauCov, sCov)
theta = [x; g; tau; s];

if nargin == 8
    thetaCov = blkdiag(xCov, gCov, tauCov, sCov);
end
end