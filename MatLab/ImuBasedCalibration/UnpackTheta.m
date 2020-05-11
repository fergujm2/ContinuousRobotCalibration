function [x, g, tau, s, xCov, gCov, tauCov, sCov] = UnpackTheta(theta, thetaCov)

[~, numParams] = GetCalibInfo();

x = theta(1:numParams);
g = theta((numParams + 1):(numParams + 3));
tau = theta(numParams + 4);
s = theta((numParams + 5):end);

if nargin == 2
    xCov = thetaCov(1:numParams,1:numParams);
    gCov = thetaCov((numParams + 1):(numParams + 3),(numParams + 1):(numParams + 3));
    tauCov = thetaCov((numParams + 4),(numParams + 4));
    sCov = thetaCov((numParams + 5):end,(numParams + 5):end);
end
end