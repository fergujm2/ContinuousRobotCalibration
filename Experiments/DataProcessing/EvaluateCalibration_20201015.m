clear;

%% Load and preprocess

testDir = '20201015_FullTest';
filename = 'DiscreteEvaluation';

[tRobot, q, ~, ~, tTracker, p, r] = loadData(testDir, filename);

% Remove the first few seconds of data
t0Robot = 3;
t0Tracker = 12;

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

numCalibPts = 100;

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

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
eNominal = zeros(numParamsTotal,1);

indBase = [1, 2, 4, 6, 8, 10];

eNominal(indBase) = eStandard(indBase);
eNominal((end - 5):end) = eStandard((end - 5):end);

theta = [0.000534908079141987;0.000461592568879848;0.00331849760791170;0.000921630443250510;0.00343580920699825;-0.000825872940559778;-0.000890663664864104;0.00645040010863206;0.000610676330713454;0.0271787117529264;0.0766415741401435;0.0198372612878607;0.0539670630661763;-0.0444284918572470;-0.105469392245139;0.0125604219751355;0.0312766682679821;-0.00162883690987939;3.14407347324211;0.0371879701184464;0.0176350216127774;0.993536402302457;0.976817711014999;0.982537253811140;-0.152164677272775;0.645191916434193;-0.166752971504286;0.000572355285222221;-0.00365027183079298;-8.87480295924340e-05;3.16342585963228;0.0289281947242712;0.0153443184878508;1.02056278921285;1.02632281674369;1.02136311861539;-0.00242707413590995;-0.00294778098009042;-0.000638239092467370];

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

figure(3);
clf;

pErrorAll = [pErrorNominal; pErrorStandard; pErrorCalib];
rErrorAll = [rErrorNominal; rErrorStandard; rErrorCalib];

xLimPos = [0, max(pErrorAll)];
xLimRot = [0, max(rErrorAll)];

subplot(3,2,1);
histogram(pErrorNominal);
title('Position Error, Nominal');
xlim(xLimPos);

subplot(3,2,3);
histogram(pErrorStandard);
title('Position Error, Standard Calibration');
xlim(xLimPos);

subplot(3,2,5);
histogram(pErrorCalib);
title('Position Error, Our Calibration');
xlim(xLimPos);
xlabel('error (mm)');

subplot(3,2,2);
histogram(rErrorNominal);
title('Rotation Error, Nominal');
xlim(xLimRot);

subplot(3,2,4);
histogram(rErrorStandard);
title('Rotation Error, Standard Calibration');
xlim(xLimRot);

subplot(3,2,6);
histogram(rErrorCalib);
title('Rotation Error, Our Calibration');
xlim(xLimRot);
xlabel('error (deg)');

%% Determine ground truth functions for a,w of the sensors

testDir = '20201015_FullTest';
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

rowsToKeep = and(tRobot > 60, tRobot < 160);

tRobot = tRobot(rowsToKeep);
pSensorTempCalib = pSensor(rowsToKeep,:);

obj = @(tau) sum(sum((pSensorTempCalib - pSensorTruth(tRobot - tau)).^2));

options = optimset('Display', 'iter', 'TolX', 1e-6);

tauRobotToTracker = fminbnd(obj, -14, -10, options);

figure(4);
clf;
hold on;

plot(tRobot, pSensorTempCalib);
plot(tRobot, pSensorTruth(tRobot - tauRobotToTracker));
title('Temporal Calibration of Tracker');

%% Compare measured values to truth and predicted functions

rowsToKeep = and(tImu > 60, tImu < 160);

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

tauImuToRobot = -8.05;
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

figure(6);
clf;

windowSize = 300;

subplot(1,2,1);
hold on;
plot(tImu, sqrt(sum(movmean(wSensorTruthS - wSensorMeasNom, windowSize, 'Endpoints', 'fill').^2,2)));
plot(tImu, sqrt(sum(movmean(wSensorTruthS - wSensorMeas, windowSize, 'Endpoints', 'fill').^2,2)));
ylabel('norm(e_w)');
legend('Nominal', 'Calibrated');
title('Sensor Angular Velocity Errors');

subplot(1,2,2);
hold on;
plot(tImu, sqrt(sum(movmean(aSensorTruthS - aSensorMeasNom, windowSize, 'Endpoints', 'shrink').^2,2)));
plot(tImu, sqrt(sum(movmean(aSensorTruthS - aSensorMeas, windowSize, 'Endpoints', 'shrink').^2,2)));
ylabel('norm(e_a)');
legend('Nominal', 'Calibrated');
title('Sensor Specific Force Errors');


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

    % Remove time offset
    tRobot = tRobot - tRobot(1);
    tTracker = tTracker - tTracker(1);
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