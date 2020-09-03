function theta = GetThetaNominal()

[~, numParams] = GetRobotCalibInfo();

x1 = zeros(numParams - 3,1);
x2 = [0.02; 0.02; 0.01];
x = [x1; x2];

gx = 0;
gy = 0;
g = [gx; gy];

tau = -0.15;

alphA = zeros(3,1);
ra = rotm2eul(eye(3))';
ka = ones(3,1);
ba = zeros(3,1);

alphW = zeros(3,1);
rw = rotm2eul(eye(3))';
kw = ones(3,1);
bw = zeros(3,1);

theta = PackTheta(x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw);
end


