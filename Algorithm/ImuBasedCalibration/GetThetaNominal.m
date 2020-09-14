function theta = GetThetaNominal()

[~, numParams] = GetRobotCalibInfo();

% All of the robot kinematic errors minus the ones used for IMU positioning
x1 = zeros(numParams - 3,1);

% Robot parameters used to position the IMU; measured with calipers
x2 = [0.0252; 0.0783; 0.0182];

% Robot parameters
x = [x1; x2];

% The gravity direction
gx = 0;
gy = 0;
g = [gx; gy];

% Time offset between robot and IMU, guessed based on previous data
tau = -0.15;

% Axis misalignments of accellerometer
alphA = zeros(3,1);
% Rotation of the accellerometer, guessed based on video
ra = [pi; 0; 0];
% Scaling of the accellerometer data
ka = ones(3,1);
% Zero offsets (biases) of the accellerometer sensor
ba = zeros(3,1);

% Gyroscope parameters, similar to the accellerometer parameters
alphW = zeros(3,1);
rw = [pi; 0; 0];
kw = ones(3,1);
bw = zeros(3,1);

theta = PackTheta(x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw);
end


