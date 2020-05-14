function [theta, thetaCov] = GetNominalTheta()

[~, numParams] = GetRobotCalibInfo();

x = zeros(numParams,1);
xCov1 = (0.01)^2*eye(numParams - 6);
xCov2 = (0.01)^2*eye(3);
xCov3 = (0.1)^2*eye(3);
xCov = blkdiag(xCov1, xCov2, xCov3);

g = [0; 0; 9.81];
% g = [0; 0; 0];
gCov = (0.01)^2*eye(3);

tau = 0;
tauCov = (0.01)^2;

s = [1; 1];
sCov = (1e-2)^2*eye(2);

[theta, thetaCov] = PackTheta(x, g, tau, s, xCov, gCov, tauCov, sCov);
end


