function [theta, thetaCov] = GetThetaNominal()

[~, numParams, ~, paramsMm, paramsDeg] = GetRobotCalibInfo();

% All of the robot kinematic errors minus the ones used for IMU positioning
x1 = zeros(numParams - 3,1);

% Robot parameters used to position the IMU; measured with calipers
x2 = [0.0252; 0.0783; 0.0182];

% Robot parameters
x = [x1; x2];

xCov = zeros(size(x));
xCov(paramsMm) = (1/1000)^2;
xCov(paramsDeg) = deg2rad(1)^2;

% The gravity direction
gx = 0;
gy = 0;

g = [gx; gy];
gCov = 0.5.^2*ones(size(g));

% Time offset between robot and IMU, guessed based on previous data
tau = -0.15;
tauCov = 1^2;

% Axis misalignments of accellerometer
alphA = zeros(3,1);
alphACov = deg2rad(1.5)^2*ones(size(alphA));

% Rotation of the accellerometer, guessed based on video
ra = [pi; 0; 0];
raCov = deg2rad(5)^2*ones(size(ra));

% Scaling of the accellerometer data
ka = ones(3,1);
kaCov = 0.1^2*ones(size(ka));

% Zero offsets (biases) of the accellerometer sensor
ba = zeros(3,1);
baCov = 2^2*ones(size(ba));

% Gyroscope parameters, similar to the accellerometer parameters
alphW = zeros(3,1);
alphWCov = deg2rad(2)^2*ones(size(alphW));

rw = [pi; 0; 0];
rwCov = deg2rad(5)^2*ones(size(rw));

kw = ones(3,1);
kwCov = 0.1^2*ones(size(kw));

bw = zeros(3,1);
bwCov = deg2rad(5)^2*ones(size(bw));

theta = PackTheta(x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw);
thetaCov = diag([xCov; gCov; tauCov; alphACov; raCov; kaCov; baCov; alphWCov; rwCov; kwCov; bwCov]);
end


