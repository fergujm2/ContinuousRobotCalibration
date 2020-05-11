function [theta, thetaCov] = GetNominalTheta()

[~, numParams] = GetCalibInfo();

x = zeros(numParams,1);
xCov1 = (0.001)^2*eye(numParams - 6);
xCov2 = (0.01)^2*eye(6);
xCov = blkdiag(xCov1, xCov2);

g = [0; 0; 9.81];
gCov = (0.01)^2*eye(3);

tau = 0;
tauCov = (0.01)^2;

s = [1; 1];
sCov = (1e-1)^2*eye(2);

[theta, thetaCov] = PackTheta(x, g, tau, s, xCov, gCov, tauCov, sCov);
end


