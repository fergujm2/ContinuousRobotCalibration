clear;

%% Load and preprocess

testDir = '20200922_FullTest';
filename = 'DiscreteEvaluation';

[tRobot, q, ~, ~, tTracker, p, r] = loadData(testDir, filename);

% Remove the first few seconds of data
t0Robot = 1;
t0Tracker = 8;

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

numPts = 249;
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

theta = [0.00157659711989707;-0.000452996765565972;0.00301310931346353;-0.000226454500731232;0.00484411567575043;-0.000344582098489011;0.000538569899705249;0.00708770109773747;-0.000295307191808682;0.0229513835859175;0.0766559983963260;0.0162932199947115;0.0516734902000655;-0.0400651893331261;-0.123162032137164;0.0133081449122899;0.0318179449598285;-0.000368406986906460;3.15658216323371;0.00560892437729537;0.0193436847637735;0.992760782694775;0.976699563585498;0.983267015003733;-0.146243678922297;0.611202398378770;-0.162078888788502;0.000745341154380688;-0.00263032660505044;-0.00132256696475529;3.16396739060705;0.0311834368055561;0.0165435479020167;1.02290274196971;1.02880562944994;1.02233308772391;-0.00227823625526643;-0.00266896126328930;-0.000802708323917289];
% theta = [0.00471786475682781;-5.74775350546409e-05;0.00273120304780113;-0.00106701863478072;0.00305416451289278;-0.00130752770650629;0.00455448433951046;0.00559307142566874;-0.00501035270717574;0.0269709758222661;0.0773029754365893;0.0192241131081616;0.0359869187100298;-0.0299848594542212;-0.125247079527767;0.0149104397001889;0.0318769581326291;0.00386898715213703;3.15827423928983;0.00624255249017475;0.0208414414348305;0.992695512612497;0.978249643898521;0.984894403965590;-0.147252084329022;0.613363468727430;-0.149377847908080;0.000731427058510594;-0.00378689223441680;-0.00814733055413123;3.16392467370188;0.0327519821132096;0.0238480475961099;1.02190581729854;1.02751271188899;1.02161490303494;-0.00225915720009621;-0.00254706529456161;-0.000671247105689072];
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

testDir = '20200922_FullTest';
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
[y, C] = LsqFitVectorSpline(pSensor, tTracker, d, floor(length(tTracker)/65));
pSensorTruth = @(t) EvalVectorSpline(y, C, d, t);

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

aSensorTruth = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

[y, C] = LsqFitVectorSpline(rSensor, tTracker, d, floor(length(tTracker)/65));
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

rowsToKeep = and(tRobot > 25, tRobot < 125);

tRobot = tRobot(rowsToKeep);
pSensorTempCalib = pSensor(rowsToKeep,:);

obj = @(tau) sum(sum((pSensorTempCalib - pSensorTruth(tRobot - tau)).^2));

options = optimset('Display', 'iter', 'TolX', 1e-6);

tauRobotToTracker = fminbnd(obj, -8, -4, options);

figure(4);
clf;
hold on;

plot(tRobot, pSensorTempCalib);
plot(tRobot, pSensorTruth(tRobot - tauRobotToTracker));
title('Temporal Calibration of Tracker');

%% Compare measured values to truth and predicted functions

% TODO For tomorrow: One thing that might be wrong is the biases bw and ba
% may be used incorrectly somehow.  It's not showing up in the ang vel
% plots because bw is so small, but ba is not small. That may shed some
% light on the issue. One more thing to check: just use
% ImuMeasurementEquation directly to compare predicted z vs measured z to
% make sure they match up. If not, then clearly the calibration didn't
% work.

rowsToKeep = and(tImu > 25, tImu < 125);

tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

thetaNominal = GetThetaNominal();
[~, gNominal, ~, alphANominal, raNominal, kaNominal, baNominal, alphWNominal, rwNominal, kwNominal, bwNominal] = UnpackTheta(thetaNominal);

Ta = [1, -alphA(1), alphA(2); 0, 1, -alphA(3); 0, 0, 1];
Ka = diag(ka);
Ra = eul2rotm(ra');

aAccel = inv((Ka / Ta)*Ra)*(z(:,1:3)' - ba); % This includes gravity

Ta = [1, -alphANominal(1), alphANominal(2); 0, 1, -alphANominal(3); 0, 0, 1];
Ka = diag(kaNominal);
Ra = eul2rotm(raNominal');

aAccelNominal = inv((Ka / Ta)*Ra)*(z(:,1:3)' - baNominal); % This includes gravity

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
gTrackerNominal = R0*gNominal;

aSensorMeas = zeros(size(aAccel));
aSensorMeasNominal = zeros(size(aAccel));

rSensor = rSensorTruth(tImu - tauRobotToTracker + tauImuToRobot);

numMeas = length(tImu);

for iii = 1:numMeas
    aSensorMeas(:,iii) = quat2rotm(rSensor(iii,:))*aAccel(:,iii);
    aSensorMeasNominal(:,iii) = quat2rotm(rSensor(iii,:))*aAccelNominal(:,iii);
end

aSensorMeas = aSensorMeas' + gTracker';
aSensorMeasNominal = aSensorMeasNominal' + gTrackerNominal';

aSensorMeas = movmean(aSensorMeas, 50);
aSensorMeasNominal = movmean(aSensorMeasNominal, 50);

Tw = [1, -alphW(1), alphW(2); 0, 1, -alphW(3); 0, 0, 1];
Kw = diag(kw);
Rw = eul2rotm(rw');

wGyroMeas = inv((Kw / Tw)*Rw)*(z(:,4:6)' - bw);

Tw = [1, -alphWNominal(1), alphWNominal(2); 0, 1, -alphWNominal(3); 0, 0, 1];
Kw = diag(kwNominal);
Rw = eul2rotm(rwNominal');

wGyroMeasNominal = inv((Kw / Tw)*Rw)*(z(:,4:6)' - bwNominal);


wSensorMeas = zeros(size(wGyroMeas));
wSensorMeasNominal = zeros(size(wGyroMeas));

for iii = 1:numMeas
    wSensorMeas(:,iii) = quat2rotm(rSensor(iii,:))*wGyroMeas(:,iii);
    wSensorMeasNominal(:,iii) = quat2rotm(rSensor(iii,:))*wGyroMeasNominal(:,iii);
end

wSensorMeas = wSensorMeas';
wSensorMeasNominal = wSensorMeasNominal';

wSensorMeas = movmean(wSensorMeas, 50);
wSensorMeasNominal = movmean(wSensorMeasNominal, 50);

wSensorKinS = wSensorKin(tImu + tauImuToRobot);
aSensorKinS = aSensorKin(tImu + tauImuToRobot);

wSensorTruthS = wSensorTruth(tImu - tauRobotToTracker + tauImuToRobot);
aSensorTruthS = aSensorTruth(tImu - tauRobotToTracker + tauImuToRobot);

figure(5);
clf;

subplot(3,2,1);
hold on;
plot(tImu, wSensorTruthS(:,1));
plot(tImu, wSensorKinS(:,1));
plot(tImu, wSensorMeas(:,1), '.', 'MarkerSize', 2);
ylabel('w_x');
title('End Effector Angular Velocity');

subplot(3,2,3);
hold on;
plot(tImu, wSensorTruthS(:,2));
plot(tImu, wSensorKinS(:,2));
plot(tImu, wSensorMeas(:,2), '.', 'MarkerSize', 2);
ylabel('w_y');

subplot(3,2,5);
hold on;
plot(tImu, wSensorTruthS(:,3));
plot(tImu, wSensorKinS(:,3));
plot(tImu, wSensorMeas(:,3), '.', 'MarkerSize', 2);
ylabel('w_z');
xlabel('time (sec)');

subplot(3,2,2);
hold on;
plot(tImu, aSensorTruthS(:,1));
plot(tImu, aSensorKinS(:,1));
plot(tImu, aSensorMeas(:,1), '.', 'MarkerSize', 2);
ylabel('a_x');
title('End Effector Accelleration');

subplot(3,2,4);
hold on;
plot(tImu, aSensorTruthS(:,2));
plot(tImu, aSensorKinS(:,2));
plot(tImu, aSensorMeas(:,2), '.', 'MarkerSize', 2);
ylabel('a_y');

subplot(3,2,6);
hold on;
plot(tImu, aSensorTruthS(:,3));
plot(tImu, aSensorKinS(:,3));
plot(tImu, aSensorMeas(:,3), '.', 'MarkerSize', 2);
ylabel('a_z');
xlabel('time (sec)');

figure(6);
clf;

subplot(3,2,1);
hold on;
plot(tImu, wSensorTruthS(:,1) - wSensorMeasNominal(:,1));
plot(tImu, wSensorTruthS(:,1) - wSensorMeas(:,1));
ylabel('e_w_x');
title('Sensor Angular Velocity Errors');

subplot(3,2,3);
hold on;
plot(tImu, wSensorTruthS(:,2) - wSensorMeasNominal(:,2));
plot(tImu, wSensorTruthS(:,2) - wSensorMeas(:,2));
ylabel('e_w_y');

subplot(3,2,5);
hold on;
plot(tImu, wSensorTruthS(:,3) - wSensorMeasNominal(:,3));
plot(tImu, wSensorTruthS(:,3) - wSensorMeas(:,3));
ylabel('e_w_z');
xlabel('time (sec)');

subplot(3,2,2);
hold on;
plot(tImu, aSensorTruthS(:,1) - aSensorMeasNominal(:,1));
plot(tImu, aSensorTruthS(:,1) - aSensorMeas(:,1));
ylabel('e_a_x');
title('Sensor Accelleration Errors');

subplot(3,2,4);
hold on;
plot(tImu, aSensorTruthS(:,2) - aSensorMeasNominal(:,2));
plot(tImu, aSensorTruthS(:,2) - aSensorMeas(:,2));
ylabel('e_a_y');

subplot(3,2,6);
hold on;
plot(tImu, aSensorTruthS(:,3) - aSensorMeasNominal(:,3));
plot(tImu, aSensorTruthS(:,3) - aSensorMeas(:,3));
ylabel('e_a_z');
xlabel('time (sec)');



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