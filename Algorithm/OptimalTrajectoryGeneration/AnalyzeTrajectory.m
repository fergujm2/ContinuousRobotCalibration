function AnalyzeTrajectory(filename)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);

y = dataObj.y;
C = dataObj.C;
d = dataObj.d;
tSpan = dataObj.tSpan;
sampleRate = dataObj.sampleRate;

% Now, we need the end points to be zero
numZeros = 5;
C(:,(end - numZeros + 1):end) = C(:,1:numZeros);

tObs = 10:5:120;

for iii = 1:length(tObs)
    [~, thetaCov] = ComputeObservability(y, C, d, sampleRate, [3, tObs(iii)], tSpan);
    stdTheta(:,iii) = sqrt(diag(thetaCov));
    [xStd(:,iii), gStd(:,iii), tauStd(:,iii), alphAStd(:,iii), raStd(:,iii), kaStd(:,iii), baStd(:,iii), alphWStd(:,iii), rwStd(:,iii), kwStd(:,iii), bwStd(:,iii)] = UnpackTheta(stdTheta(:,iii));
%     stdThetaOld(:,iii) = sqrt(diag(dataObj.thetaCov(:,:,iii+1)));
end

[calibBools, numParams, numParamsTotal] = GetRobotCalibInfo();

paramsMm = logical(repmat([ones(3,1); zeros(3,1)], numParamsTotal/6, 1));
paramsMm = paramsMm(calibBools);
paramsDeg = not(paramsMm);

xStdMm = xStd(paramsMm,:).*1000;
xStdDeg = rad2deg(xStd(paramsDeg,:));

figure(1);
clf;

subplot(1,2,1);
plot(tObs, xStdMm');
xlabel('t (sec)');
ylabel('Standard Deviation (mm)');

subplot(1,2,2);
plot(tObs, xStdDeg');
xlabel('t (sec)');
ylabel('Standard Deviation (deg)');

figure(2);
clf;

subplot(1,2,1);
plot(tObs, gStd(1:2,:)');
xlabel('t (sec)');
ylabel('Standard Deviation (m/s/s)');

subplot(1,2,2);
plot(tObs, tauStd);
xlabel('t (sec)');
ylabel('Standard Deviation (sec)');

figure(3);
clf;

subplot(2,2,1);
plot(tObs, alphAStd');
xlabel('t (sec)');
ylabel('Standard Deviation (deg)');

subplot(2,2,2);
plot(tObs, raStd');
xlabel('t (sec)');
ylabel('Standard Deviation (deg)');

subplot(2,2,3);
plot(tObs, kaStd');
xlabel('t (sec)');
ylabel('Standard Deviation (unitless)');

subplot(2,2,4);
plot(tObs, baStd');
xlabel('t (sec)');
ylabel('Standard Deviation (m/s/s)');

figure(4);
clf;

subplot(2,2,1);
plot(tObs, alphWStd');
xlabel('t (sec)');
ylabel('Standard Deviation (deg)');

subplot(2,2,2);
plot(tObs, rwStd');
xlabel('t (sec)');
ylabel('Standard Deviation (deg)');

subplot(2,2,3);
plot(tObs, kwStd');
xlabel('t (sec)');
ylabel('Standard Deviation (unitless)');

subplot(2,2,4);
plot(tObs, bwStd');
xlabel('t (sec)');
ylabel('Standard Deviation (rad/s)');

end