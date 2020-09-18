clear;

%% Load and preprocess

testDir = '20200915_FullTest';
filename = 'DiscreteEvaluation';

[tRobot, q, tTracker, p, r] = loadData(testDir, filename);

% Remove the first few seconds of data
t0Robot = 4;
t0Tracker = 9.75;

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

numPts = 149;
T = 5;

[q, p, r] = extractAveragePauses(tRobot, q, tTracker, p, r, numPts, T);
R = quat2rotm(r);

%% Split data into subsets and compute standard robot calibration

numCalibPts = 50;

calibInd = 1:numCalibPts;
evalInd = (numCalibPts + 1):numPts;

qCalib = q(calibInd,:);
pCalib = p(calibInd,:);
RCalib = R(:,:,calibInd);

qEval = q(evalInd,:);
pEval = p(evalInd,:);
REval = R(:,:,evalInd);

eStandard = ComputeStandardCalibration(qCalib, pCalib, RCalib);

%% Compute accuracy

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
eNominal = zeros(numParamsTotal,1);

indBase = [1, 2, 4, 6, 8, 10];
eNominal(indBase) = eStandard(indBase);
eNominal((end - 5):end) = eStandard((end - 5):end);

theta = [0.00157626917060091;0.000582893887031478;0.00250997289699063;0.000103136659398190;0.00342141255329507;-0.00144021483334639;0.000880835216794323;0.00624163671233383;-0.00178378897514887;0.0260060601817521;0.0757400322100505;0.0183177628561811;-0.0216546268528506;-0.0158225163964553;-0.0549863990878930;0.0129890207108208;0.0312387147506419;0.000103017305392591;3.15645625722441;0.00446953696565359;0.0205092149125808;0.993326242387823;0.977009283385618;0.984364715319519;-0.161943688186400;0.667828328952695;-0.172283898554833;-0.000342885949910048;-0.00307064532945712;-0.00286673319564627;3.16160777538487;0.0316864549325502;0.0186384544191669;1.01938535179610;1.02625601142728;1.02057471921373;-0.00231074822962883;-0.00252065508897620;-0.000729391760351169];

[x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw] = UnpackTheta(theta);

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

function [tRobot, q, tTracker, p, r] = loadData(testDir, filename)
    dataDir = fullfile('..', testDir, 'DataProcessed');
    dataFullfilename = fullfile(dataDir, filename);
    dataFullfilenameTracker = fullfile(dataDir, [filename, 'Tracker']);

    serialDataObj = load(dataFullfilename);
    trackerDataObj = load(dataFullfilenameTracker);

    q = serialDataObj.q; % Robot joint values
    tRobot = serialDataObj.tRobot;
    p = trackerDataObj.p; % Tracker position measurements
    r = trackerDataObj.q; % Tracker quaternion measurements
    tTracker = trackerDataObj.t;

    % Remove time offset
    tRobot = tRobot - tRobot(1);
    tTracker = tTracker - tTracker(1);
end