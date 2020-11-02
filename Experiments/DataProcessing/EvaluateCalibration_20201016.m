clear;

%% Load and preprocess

testDir = '20201016_FullTest';
filename = 'DiscreteEvaluation';

[tRobot, q, ~, ~, tTracker, p, r] = loadData(testDir, filename);

% Remove the first few seconds of data
t0Robot = 5;
t0Tracker = 11;

rowsToKeep = tRobot > t0Robot;
tRobot = tRobot(rowsToKeep);
q = q(rowsToKeep,:);

rowsToKeep = tTracker > t0Tracker;
tTracker = tTracker(rowsToKeep);
p = p(rowsToKeep,:);
r = r(rowsToKeep,:);

% Roughly remove the time offset again
tRobot = tRobot - tRobot(1);
tTracker = tTracker - tTracker(1);

figure(1);
clf;

subplot(2,1,1);
plot(tRobot, q);
title('Robot Joint Values');
xlabel('time (sec)');

subplot(2,1,2);
plot(tTracker, p);
title('Tracker Position');
xlabel('time (sec)');

%% Extract the average of the data at the discrete pauses

numPts = 250;
TSensor = 5;

[q, p, r] = extractAveragePauses(tRobot, q, tTracker, p, r, numPts, TSensor);
R = quat2rotm(r);

%% Split data into subsets and compute standard robot calibration

numCalibPts = 70;

calibInd = 1:numCalibPts;
evalInd = (numCalibPts + 1):numPts;

qCalib = q(calibInd,:);
pCalib = p(calibInd,:);
RCalib = R(:,:,calibInd);

qEval = q(evalInd,:);
pEval = p(evalInd,:);
REval = R(:,:,evalInd);

eStandard = ComputeStandardCalibration(qCalib, pCalib, RCalib);

%% Compute robot accuracy

calibFilename = fullfile('..', testDir, 'OutputCalibrations', 'Calibration_0To300');
calibObj = load(calibFilename);

theta = calibObj.thetaStar;

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
eNominal = zeros(numParamsTotal,1);

indBase = [1, 2, 4, 6, 8, 10];

eNominal(indBase) = eStandard(indBase);
eNominal((end - 5):end) = eStandard((end - 5):end);

[x, g, tauImuToRobot, alphA, ra, ka, ba, alphW, rw, kw, bw] = UnpackTheta(theta);

eCalib = eNominal;
eCalib(calibBools) = x;
eCalib((end - 5):end) = eStandard((end - 5):end);

table(eNominal, eStandard, eCalib)

[pErrorNominal, rErrorNominal] = ComputeRobotAccuracy(qEval, eNominal, pEval, REval);
[pErrorStandard, rErrorStandard] = ComputeRobotAccuracy(qEval, eStandard, pEval, REval);
[pErrorCalib, rErrorCalib] = ComputeRobotAccuracy(qEval, eCalib, pEval, REval);

pErrorNominal = sqrt(sum(pErrorNominal.^2, 2))*1000;
rErrorNominal = rad2deg(sqrt(sum(rErrorNominal.^2, 2)));
pErrorStandard = sqrt(sum(pErrorStandard.^2, 2))*1000;
rErrorStandard = rad2deg(sqrt(sum(rErrorStandard.^2, 2)));
pErrorCalib = sqrt(sum(pErrorCalib.^2, 2))*1000;
rErrorCalib = rad2deg(sqrt(sum(rErrorCalib.^2, 2)));

h = figure(3);
h.Color = [1,1,1];
clf;

subplot(1,2,1);
hold on;

histogram(pErrorNominal, 'BinWidth', .275);
histogram(pErrorCalib, 'BinWidth', .275);
% histogram(pErrorStandard);
ax = gca;
ax.FontSize = 8;

xlim([0, 3.5]);
xlabel('Position Error (mm)', 'Interpreter', 'Latex');

subplot(1,2,2);
hold on;

histogram(rErrorNominal, 'BinWidth', .07);
histogram(rErrorCalib, 'BinWidth', .07);
% histogram(rErrorStandard);
ax = gca;
ax.FontSize = 8;

legend('Nominal', 'Calibrated', 'Interpreter', 'Latex');
xlim([0, .9]);
xlabel('Rotation Error ($^{\circ}$)', 'Interpreter', 'Latex');

saveFigurePdf([3.5, 2]);

%% Determine ground truth functions for a,w of the sensors

filename = 'ContinuousEvaluation';

[tRobot, q, tImu, z, tTracker, p, r] = loadData(testDir, filename);

pSensor = zeros(size(p));
rSensor = zeros(size(r));

toolToEe = ErrorTransform(eStandard((end - 5):end)', false);
sensorToEe = ErrorTransform([x((end - 2):end); [0;0;0]]', false);

for iii = 1:length(tTracker)
    TSensor = trvec2tform(p(iii,:))*quat2tform(r(iii,:))*inv(toolToEe)*sensorToEe;
    
    pSensor(iii,:) = TSensor(1:3,4);
    rSensor(iii,:) = rotm2quat(TSensor(1:3,1:3));
end

rSensor = unwrapQuat(rSensor); % Make quaternion continuous

d = 5;
[y, C] = LsqFitVectorSpline(pSensor, tTracker, d, floor(length(tTracker)/20));
pSensorTruth = @(t) EvalVectorSpline(y, C, d, t);

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

aSensorTruth = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

[y, C] = LsqFitVectorSpline(rSensor, tTracker, d, floor(length(tTracker)/20));
rSensorTruth = @(t) EvalVectorSpline(y, C, d, t);

[yd, Cd, dd] = DerVectorSpline(y, C, d);
rSensorDotTruth = @(t) EvalVectorSpline(yd, Cd, dd, t);

wSensorTruth = @(t) quatToAngularVelocity(rSensorTruth(t), rSensorDotTruth(t));

% We're going to the sensor, not the tool now
eSensor = eStandard;
eSensor((end - 5):end) = 0;
eSensor((end - 5):(end - 3)) = x((end - 2):end);

[pSensor, RSensor] = ComputeForwardKinematics(q, eSensor, false);

[y, C] = LsqFitVectorSpline(q, tRobot, d, floor(length(tRobot)/30));

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

qf = @(t) EvalVectorSpline(y, C, d, t);
qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

[~, aSensor, wSensor] = ComputeAccellerations(qf(tRobot), qDot(tRobot), qDDot(tRobot), eSensor);

aSensor = squeeze(aSensor(:,end,:))';
wSensor = squeeze(wSensor(:,end,:))';

[y, C] = LsqFitVectorSpline(aSensor, tRobot, d, floor(length(tRobot)/30));
aSensorKin = @(t) EvalVectorSpline(y, C, d, t);

[y, C] = LsqFitVectorSpline(wSensor, tRobot, d, floor(length(tRobot)/30));
wSensorKin = @(t) EvalVectorSpline(y, C, d, t);

rowsToKeep = and(tRobot > 920, tRobot < 1040);

tRobot = tRobot(rowsToKeep);
pSensorTempCalib = pSensor(rowsToKeep,:);

obj = @(tau) sum(sum((pSensorTempCalib - pSensorTruth(tRobot - tau)).^2));

options = optimset('Display', 'iter', 'TolX', 1e-6);

tauRobotToTracker = fminbnd(obj, 826 - 2, 826 + 2, options);

figure(4);
clf;
hold on;

plot(tRobot, pSensorTempCalib);
plot(tRobot, pSensorTruth(tRobot - tauRobotToTracker));
title('Temporal Calibration of Tracker');

%% Compare measured values to truth and predicted functions

rowsToKeep = and(tImu > 920, tImu < 1040);

tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

Ta = [1, 0, 0; alphA(1), 1, 0; -alphA(2), alphA(3), 1];
Ka = diag(ka);
Ra = eul2rotm(ra');

Tw = [1, 0, 0; alphW(1), 1, 0; -alphW(2), alphW(3), 1];
Kw = diag(kw);
Rw = eul2rotm(rw');

aSensorMeas = inv(Ka*Ta*Ra)*(z(:,1:3)' - ba);
wSensorMeas = inv(Kw*Tw*Rw)*(z(:,4:6)' - bw);

aSensorMeas = aSensorMeas';
wSensorMeas = wSensorMeas';

thetaNom = GetThetaNominal();
[~, ~, ~, alphANom, raNom, kaNom, baNom, alphWNom, rwNom, kwNom, bwNom] = UnpackTheta(thetaNom);

TaNom = [1, 0, 0; alphANom(1), 1, 0; -alphANom(2), alphANom(3), 1];
KaNom = diag(kaNom);
RaNom = eul2rotm(raNom');

TwNom = [1, 0, 0; alphWNom(1), 1, 0; -alphWNom(2), alphWNom(3), 1];
KwNom = diag(kwNom);
RwNom = eul2rotm(rwNom');

aSensorMeasNom = inv(KaNom*TaNom*RaNom)*(z(:,1:3)' - baNom);
wSensorMeasNom = inv(KwNom*TwNom*RwNom)*(z(:,4:6)' - bwNom);

aSensorMeasNom = aSensorMeasNom';
wSensorMeasNom = wSensorMeasNom';

wSensorTruthS = wSensorTruth(tImu - tauRobotToTracker + tauImuToRobot);
aSensorTruthS = aSensorTruth(tImu - tauRobotToTracker + tauImuToRobot);

% Get frame 0 of robot
[~, ~, ~, frames] = ComputeForwardKinematics(q(1,:), eSensor, false);
R0 = frames(1:3,1:3,1);

eBase = eSensor(indBase);

% tx ty ry rx ty ry
Ry1 = axang2rotm([0, 1, 0, eBase(3)]);
Rx = axang2rotm([1, 0, 0, eBase(4)]);
Ry2 = axang2rotm([0, 1, 0, eBase(6)]);
RInt = axang2rotm([0, 0, 1, pi/2]);

Rz = axang2rotm([0, 0, 1, eBase(6)]);

% R1 = Ry1*Rx*RInt*Ry2;
R1 = Ry1*Rx*RInt*Ry2;
R111 = Ry1*Rx*Rz;

gTracker = R0*g;

aSensorTruthS = aSensorTruthS - gTracker';

numMeas = length(tImu);
rSensor = rSensorTruth(tImu - tauRobotToTracker + tauImuToRobot);

for iii = 1:numMeas
    Ri = quat2rotm(rSensor(iii,:));
    aSensorTruthS(iii,:) = inv(Ri)*(aSensorTruthS(iii,:)');
    wSensorTruthS(iii,:) = inv(Ri)*(wSensorTruthS(iii,:)');
end

figure(5);
clf;

subplot(3,2,1);
hold on;
plot(tImu, wSensorTruthS(:,1));
plot(tImu, wSensorMeas(:,1), '.', 'MarkerSize', 2);
ylabel('w_x');
title('End Effector Angular Velocity');

subplot(3,2,3);
hold on;
plot(tImu, wSensorTruthS(:,2));
plot(tImu, wSensorMeas(:,2), '.', 'MarkerSize', 2);
ylabel('w_y');

subplot(3,2,5);
hold on;
plot(tImu, wSensorTruthS(:,3));
plot(tImu, wSensorMeas(:,3), '.', 'MarkerSize', 2);
ylabel('w_z');
xlabel('time (sec)');

subplot(3,2,2);
hold on;
plot(tImu, aSensorTruthS(:,1));
plot(tImu, aSensorMeas(:,1), '.', 'MarkerSize', 2);
ylabel('a_x');
title('End Effector Accelleration');

subplot(3,2,4);
hold on;
plot(tImu, aSensorTruthS(:,2));
plot(tImu, aSensorMeas(:,2), '.', 'MarkerSize', 2);
ylabel('a_y');

subplot(3,2,6);
hold on;
plot(tImu, aSensorTruthS(:,3));
plot(tImu, aSensorMeas(:,3), '.', 'MarkerSize', 2);
ylabel('a_z');
xlabel('time (sec)');

h = figure(6);
clf;
h.Color = [1,1,1];

windowSize = 20;

subplot(2,1,1);
hold on;
plot(tImu - tImu(1), rad2deg(sqrt(sum(movmean(wSensorTruthS - wSensorMeasNom, windowSize, 'Endpoints', 'fill').^2,2))));
plot(tImu - tImu(1), rad2deg(sqrt(sum(movmean(wSensorTruthS - wSensorMeas, windowSize, 'Endpoints', 'fill').^2,2))));
ylabel('angular velocity error ($^{\circ}$/s)', 'Interpreter', 'Latex');
ax = gca;
ax.FontSize = 8;

subplot(2,1,2);
hold on;
plot(tImu - tImu(1), sqrt(sum(movmean(aSensorTruthS - aSensorMeasNom, windowSize, 'Endpoints', 'shrink').^2,2)));
plot(tImu - tImu(1), sqrt(sum(movmean(aSensorTruthS - aSensorMeas, windowSize, 'Endpoints', 'shrink').^2,2)));
ylabel('specific force error (m/s$^2$)', 'Interpreter', 'Latex');
xlabel('time (sec)', 'Interpreter', 'Latex');
legend('Nominal', 'Calibrated', 'Interpreter', 'Latex');
ax = gca;
ax.FontSize = 8;

saveFigurePdf([3.5, 4]);

function [qEval, pEval, rEval] = extractAveragePauses(tRobot, q, tTracker, p, r, numPoints, T)

    tPause = 0:T:(T*(numPoints - 1));

    dq = sqrt(sum(diff(q).^2, 2));
    dp = sqrt(sum(diff(p).^2, 2));

    figure(2);
    clf;

    subplot(2,1,1);
    hold on;

    plot(tRobot(1:(end-1)), dq, '.k');
    plot(tPause, zeros(size(tPause)), '.', 'MarkerSize', 20);
    legend('norm(dq)', 'pauses');
    title('Pauses and Differences in Joint Values');

    subplot(2,1,2);
    hold on;

    plot(tTracker(1:(end-1)), dp, '.k');
    plot(tPause, zeros(size(tPause)), '.', 'MarkerSize', 20);
    legend('norm(dp)', 'pauses');
    title('Pauses and Differences in Tracker Position');

    qEval = zeros(numPoints, 6);
    pEval = zeros(numPoints, 3);
    rEval = zeros(numPoints, 4);

    for iii = 1:numPoints
        a = tPause(iii);
        b = a + 1;

        robotInd = and(tRobot > a, tRobot < b);
        trackerInd = and(tTracker > a, tTracker < b);

        qData = q(robotInd,:);
        pData = p(trackerInd,:);
        rData = r(trackerInd,:);

        if any(cov(qData) > 1e-6)
            warning('qData varies during pause.');
        end

        if any(cov(pData) > 1e-1)
            warning('pData varies during pause.');
        end

        qEval(iii,:) = mean(qData,1);
        pEval(iii,:) = mean(pData,1);
        rEval(iii,:) = AverageRotations(rData);
    end
end

function [tRobot, q, tImu, z, tTracker, p, r] = loadData(testDir, filename)
    dataDir = fullfile('..', testDir, 'DataProcessed');
    dataFullfilename = fullfile(dataDir, filename);
    dataFullfilenameTracker = fullfile(dataDir, [filename, 'Tracker']);

    serialDataObj = load(dataFullfilename);
    trackerDataObj = load(dataFullfilenameTracker);

    q = serialDataObj.q; % Robot joint values
    tRobot = serialDataObj.tRobot;
    tImu = serialDataObj.tImu;
    z = serialDataObj.z;

    p = trackerDataObj.p; % Tracker position measurements
    r = trackerDataObj.q; % Tracker quaternion measurements
    tTracker = trackerDataObj.t;
end

function w = quatToAngularVelocity(q, qDot)
    wq = 2*quatmultiply(qDot, quatconj(q)); % Space frame
%     wq = 2*quatmultiply(quatconj(q), qDot); % Body frame
    w = wq(:,2:end);
end

function q = unwrapQuat(q)
    for iii = 2:size(q,1)
        qxyzOld = q(iii-1,2:end);
        qxyz = q(iii,2:end);
    
        qxyzOldDir = qxyzOld./norm(qxyzOld);
        qxyzDir = qxyz./norm(qxyz);

        signFlipped = dot(qxyzOldDir, qxyzDir) < -0.9;

        if signFlipped
            q(iii,:) = -q(iii,:);
        end
    end
end

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end