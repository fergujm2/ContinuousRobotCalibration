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
T = 5;

[q, p, r] = extractAveragePauses(tRobot, q, tTracker, p, r, numPts, T);
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

pAccelMeas = zeros(size(p));
pEeMeas = zeros(size(p));

Ra = eul2rotm(ra');
Rw = eul2rotm(rw');

TAccelToEe = trvec2tform(x((end - 2):end)')*rotm2tform(Ra);
TGyroToEe = trvec2tform(x((end - 2):end)')*rotm2tform(Rw);

RGyroMeas = zeros(3, 3, length(tTracker));
toolToEe = ErrorTransform(eStandard((end - 5):end)', false);

for iii = 1:length(tTracker)
    TEe = trvec2tform(p(iii,:))*quat2tform(r(iii,:))*inv(toolToEe);
    TAccel = TEe*TAccelToEe;
    TGyro = TEe*TGyroToEe;
    
    pEeMeas(iii,:) = TEe(1:3,4);
    pAccelMeas(iii,:) = TAccel(1:3,4);
    RGyroMeas(:,:,iii) = TGyro(1:3,1:3);
end

% Make quaternion continuous
rGyroMeas = rotm2quat(RGyroMeas);

for iii = 2:length(tTracker)
    qxyzOld = rGyroMeas(iii-1,2:end);
    qxyz = rGyroMeas(iii,2:end);
    
    qxyzOldDir = qxyzOld./norm(qxyzOld);
    qxyzDir = qxyz./norm(qxyz);
    
    signFlipped = dot(qxyzOldDir, qxyzDir) < -0.9;
    
    if signFlipped
        rGyroMeas(iii,:) = -rGyroMeas(iii,:);
    end
end

d = 5;
[y, C] = LsqFitVectorSpline(pEeMeas, tTracker, d, floor(length(tTracker)/30));
pEeTruth = @(t) EvalVectorSpline(y, C, d, t);

d = 5;
[y, C] = LsqFitVectorSpline(pAccelMeas, tTracker, d, floor(length(tTracker)/30));
pAccelTruth = @(t) EvalVectorSpline(y, C, d, t);

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

aAccelTruth = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

[y, C] = LsqFitVectorSpline(rGyroMeas, tTracker, d, floor(length(tTracker)/30));
rGyroTruth = @(t) EvalVectorSpline(y, C, d, t);

[yd, Cd, dd] = DerVectorSpline(y, C, d);
rGyroDotTruth = @(t) EvalVectorSpline(yd, Cd, dd, t);

wGyroTruth = @(t) quatToAngularVelocity(rGyroTruth(t), rGyroDotTruth(t));

% Position of tip of robot
eEe = eStandard;
eEe((end - 5):end) = 0;

pEe = ComputeForwardKinematics(q, eEe, false);

rowsToKeep = and(tRobot > 25, tRobot < 125);

tRobot = tRobot(rowsToKeep);
pEe = pEe(rowsToKeep,:);

obj = @(tau) sum(sum((pEe - pEeTruth(tRobot - tau)).^2));

options = optimset('Display', 'iter', 'TolX', 1e-6);

tauRobotToTracker = fminbnd(obj, -8, -4, options);

pAccelTruth = @(t) pAccelTruth(t - tauRobotToTracker);

figure(4);
clf;
hold on;

plot(tRobot, pEe);
plot(tRobot, pEeTruth(tRobot - tauRobotToTracker));
title('Temporal Calibration of Tracker');

wGyroTruth = @(t) wGyroTruth(t - tauRobotToTracker + tauImuToRobot);
aAccelTruth = @(t) aAccelTruth(t - tauRobotToTracker + tauImuToRobot);

%% Compare measured values to truth function

rowsToKeep = and(tImu > 25, tImu < 125);

tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

Ta = [1, -alphA(1), alphA(2); 0, 1, -alphA(3); 0, 0, 1];
Ka = diag(ka);

Tw = [1, -alphW(1), alphW(2); 0, 1, -alphW(3); 0, 0, 1];
Kw = diag(kw);

omeg = inv(Rw)*(inv((Kw / Tw)*Rw)*(z(:,4:6)' - bw));

% TODO Tomorrow: figure out which frame we're expressing these in, also
% incorporate the estimate of g into the below alph
alph = inv(Ra)*(inv((Ka / Ta)*Ra)*(z(:,1:3)' - ba));

% omeg = (Rw*omeg)';

figure(5);
clf;
hold on;

plot(tImu, wGyroTruth(tImu));
plot(tImu, omeg);
title('Angular Velocity Truth vs. Measured');

figure(6);
clf;
hold on;

plot(tImu, aAccelTruth(tImu));
plot(tImu, alph);
title('Angular Velocity Truth vs. Measured');

% eBase = eStandard(indBase);
% 
% tx = eBase(1);
% ty1 = eBase(2);
% ry1 = eBase(3);
% rx = eBase(4);
% ty2 = eBase(5);
% ry2 = eBase(6);
% 
% T1 = trvec2tform([tx, 0, 0]);
% T2 = trvec2tform([0, ty1, 0]);
% T3 = axang2tform([0, 1, 0, ry1]);
% T4 = axang2tform([1, 0, 0, rx]);
% T5 = trvec2tform([0, ty2, 0]);
% T6 = axang2tform([0, 1, 0, ry2]);
% 
% baseToTracker = T6*T5*T4*T3*T2*T1;

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
%     wq = 2*quatmultiply(qDot, quatconj(q));
    wq = 2*quatmultiply(quatconj(q), qDot);
    w = wq(:,2:end);
end