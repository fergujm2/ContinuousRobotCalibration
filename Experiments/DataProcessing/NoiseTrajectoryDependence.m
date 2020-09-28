clear;

%% Determine zCov without any robot motion

testDir = '20200922_FullTest';
filename = 'DiscreteEvaluation';

dataDir = fullfile('..', testDir, 'DataProcessed');
dataFullfilename = fullfile(dataDir, filename);
serialDataObj = load(dataFullfilename);

q = serialDataObj.q; % Robot joint values
tRobot = serialDataObj.tRobot;
z = serialDataObj.z;
tImu = serialDataObj.tImu;

a = 0;
b = 60*5;

rowsToKeep = and(tRobot > a, tRobot < b);
tRobot = tRobot(rowsToKeep);
q = q(rowsToKeep,:);

rowsToKeep = and(tImu > (a + 5), tImu < (b - 5));
tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

d = 5;
[y, C] = LsqFitVectorSpline(q, tRobot, d, floor(length(tRobot)/50));

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

qf = @(t) EvalVectorSpline(y, C, d, t);
qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

[y, C] = LsqFitVectorSpline(z, tImu, d, floor(length(tImu)/30));
zf = @(t) EvalVectorSpline(y, C, d, t);

qs = qf(tRobot);
qDots = qDot(tRobot);
qDDots = qDDot(tRobot);

zs = zf(tImu);

PlotImuMeasurements(tRobot, q, qs, tImu, z, zs);

zError = z - zs;

qImu = qf(tImu);
qDotImu = qDot(tImu);

indStopped = sum(qDotImu.^2, 2) < 0.001;
zErrorStopped = zError(indStopped,:);
covBias = diag(cov(zErrorStopped))';

%% Fit model of qDot and the IMU signal noise

filename = 'Calibration';
dataFullfilename = fullfile(dataDir, filename);
serialDataObj = load(dataFullfilename);

q = serialDataObj.q; % Robot joint values
tRobot = serialDataObj.tRobot;
z = serialDataObj.z;
tImu = serialDataObj.tImu;

a = 0;
b = a + 10*60;

rowsToKeep = and(tRobot > a, tRobot < b);
tRobot = tRobot(rowsToKeep);
q = q(rowsToKeep,:);

rowsToKeep = and(tImu > (a + 1), tImu < (b - 1));
tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

d = 5;
[y, C] = LsqFitVectorSpline(q, tRobot, d, floor(length(tRobot)/50));
[yd, Cd, dd] = DerVectorSpline(y, C, d);

qf = @(t) EvalVectorSpline(y, C, d, t);
qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);

[y, C] = LsqFitVectorSpline(z, tImu, d, floor(length(tImu)/50));
zf = @(t) EvalVectorSpline(y, C, d, t);

qImu = qf(tImu);
qDotImu = qDot(tImu);

zs = zf(tImu);
zError = z - zs;

% Split data into calib and eval sets
numFit = floor(length(tImu)*0.85);
indFit = 1:numFit;
indEval = (numFit + 1):length(tImu);

theta = GetThetaNominal();

obj = @(x) abs(zError(indFit,:)) - sqrt(ComputeZCovPrior(theta, qImu(indFit,:), qDotImu(indFit,:), covBias, x(1:6), x(7:12)));

x0 = [ones(6,1); ones(6,1)];

options = optimoptions(@lsqnonlin, ...
                       'Algorithm', 'levenberg-marquardt', ...
                       'Display', 'iter', ...
                       'StepTolerance', 1e-4, ...
                       'FunctionTolerance', 1e-8);

x = lsqnonlin(obj, x0, [], [], options);

%% Plot results of fitting dependence on qDot

kaqd = x(1:6);
kwqd = x(7:12);

zCovComputed = ComputeZCovPrior(theta, qImu(indEval,:), qDotImu(indEval,:), covBias, kaqd, kwqd);
zCovMeas = ComputeZCovPost(tImu(indEval), z(indEval,:), 0.5);


figure(4);
clf;

subplot(3,2,1);
hold on;

plot(tImu(indEval), sqrt(zCovMeas(:,1)));
plot(tImu(indEval), sqrt(zCovComputed(:,1)), '-r');
ylabel('std(a_x)');

subplot(3,2,2);
hold on;

plot(tImu(indEval), sqrt(zCovMeas(:,4)));
plot(tImu(indEval), sqrt(zCovComputed(:,4)), '-r');
ylabel('std(w_x)');

subplot(3,2,3);
hold on;

plot(tImu(indEval), sqrt(zCovMeas(:,2)));
plot(tImu(indEval), sqrt(zCovComputed(:,2)), '-r');
ylabel('std(a_y)');

subplot(3,2,4);
hold on;

plot(tImu(indEval), sqrt(zCovMeas(:,5)));
plot(tImu(indEval), sqrt(zCovComputed(:,5)), '-r');
ylabel('std(w_y)');

subplot(3,2,5);
hold on;

plot(tImu(indEval), sqrt(zCovMeas(:,3)));
plot(tImu(indEval), sqrt(zCovComputed(:,3)), '-r');
ylabel('std(a_z)');

subplot(3,2,6);
hold on;

plot(tImu(indEval), sqrt(zCovMeas(:,6)));
plot(tImu(indEval), sqrt(zCovComputed(:,6)), '-r');
ylabel('std(w_z)');

% figure(5);
% clf;
% 
% subplot(3,2,1);
% hold on;
% 
% plot(tBin, sqrt(zCovBin(:,4)));
% plot(tImu(indEval), sqrt(zCov(:,4)), '-r');
% 
% ylabel('std(w_x)');
% 
% subplot(3,2,2);
% hold on;
% 
% plot(tImu(indEval), zError(indEval,4), '.', 'MarkerSize', 2);
% plot(tImu(indEval), [sqrt(zCov(:,4)), -sqrt(zCov(:,4))], '-r');
% ylabel('w_x noise');
% 
% subplot(3,2,3);
% hold on;
% 
% plot(tBin, sqrt(zCovBin(:,5)));
% plot(tImu(indEval), sqrt(zCov(:,5)), '-r');
% 
% ylabel('std(w_y)');
% 
% subplot(3,2,4);
% hold on;
% 
% plot(tImu(indEval), zError(indEval,5), '.', 'MarkerSize', 2);
% plot(tImu(indEval), [sqrt(zCov(:,5)), -sqrt(zCov(:,5))], '-r');
% ylabel('w_y noise');
% 
% subplot(3,2,5);
% hold on;
% 
% plot(tBin, sqrt(zCovBin(:,6)));
% plot(tImu(indEval), sqrt(zCov(:,6)), '-r');
% 
% ylabel('std(w_z)');
% 
% subplot(3,2,6);
% hold on;
% 
% plot(tImu(indEval), zError(indEval,6), '.', 'MarkerSize', 2);
% plot(tImu(indEval), [sqrt(zCov(:,6)), -sqrt(zCov(:,6))], '-r');
% ylabel('w_z noise');

fprintf('Statistical Model Parameters: \n');
fprintf('  Covariance while robot stopped: \n');
disp(covBias);
fprintf('  kaqd: How much each joint speed affects accel. covariance: \n');
disp(kaqd');
fprintf('  kawd: How much each joint speed affects gyro. covariance: \n');
disp(kwqd');
