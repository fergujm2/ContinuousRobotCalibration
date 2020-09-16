function AnalyzeTrajectory(filename)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);

y = dataObj.y;
C = dataObj.C;
d = dataObj.d;
tSpan = dataObj.tSpan;
sampleRate = dataObj.sampleRate;


% Now, we need the end points to be zero
numZeros = 3;
C(:,(end - numZeros + 1):end) = repmat(C(:,1), 1, numZeros);

tObs = 110:10:120;

for iii = 1:length(tObs)
    fprintf('Computing %.0f of %.0f observabilities...\n', iii, length(tObs));
    
    [~, thetaCov] = ComputeObservability(y, C, d, sampleRate, [3, tObs(iii)], []);
    
    stdTheta(:,iii) = sqrt(diag(thetaCov));
    [xStd(:,iii), gStd(:,iii), tauStd(:,iii), alphAStd(:,iii), raStd(:,iii), kaStd(:,iii), baStd(:,iii), alphWStd(:,iii), rwStd(:,iii), kwStd(:,iii), bwStd(:,iii)] = UnpackTheta(stdTheta(:,iii));
end

[calibBools, numParams, numParamsTotal] = GetRobotCalibInfo();

paramsMm = logical(repmat([ones(3,1); zeros(3,1)], numParamsTotal/6, 1));
paramsMm = paramsMm(calibBools);
paramsDeg = not(paramsMm);

xStdMm = xStd(paramsMm,:).*1000;
xStdDeg = rad2deg(xStd(paramsDeg,:));

figureDir = fullfile('Output', 'Figures');

figure(1);
clf;
title('Max Standard Deviation of Robot Parameters');

yyaxis left;
plot(tObs, max(xStdMm)');
xlabel('t (sec)');
ylabel('Max Standard Deviation of Lengths (mm)');

yyaxis right;
plot(tObs, max(xStdDeg)');
ylabel('Max Standard Deviation of Angles (deg)');

fullFilename = fullfile(figureDir, [filename, '_RobotParams']);
savefig(fullFilename);

figure(2);
clf;

subplot(1,2,1);
plot(tObs, max(gStd(1:2,:))');
title('Max STD of Gravity');
xlabel('t (sec)');
ylabel('STD (m/s/s)');

subplot(1,2,2);
plot(tObs, tauStd);
title('STD of Time Offset');
xlabel('t (sec)');
ylabel('STD (sec)');

fullFilename = fullfile(figureDir, [filename, '_Extrinsics']);
savefig(fullFilename);

figure(3);
clf;

subplot(2,2,1);
plot(tObs, rad2deg(max(alphAStd)'));
title('Max STD of Axis Misalignments');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,2);
plot(tObs, rad2deg(max(raStd)'));
title('Max STD of Orientation');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,3);
plot(tObs, max(kaStd)');
title('Max STD of Gains');
xlabel('t (sec)');
ylabel('STD (unitless)');

subplot(2,2,4);
plot(tObs, max(baStd)');
title('Max STD of Biases');
xlabel('t (sec)');
ylabel('STD (m/s/s)');

fullFilename = fullfile(figureDir, [filename, '_Accell']);
savefig(fullFilename);

figure(4);
clf;

subplot(2,2,1);
plot(tObs, rad2deg(max(alphWStd)'));
title('Max STD of Axis Misalignments');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,2);
plot(tObs, rad2deg(max(rwStd)'));
title('Max STD of Orientation');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,3);
plot(tObs, max(kwStd)');
title('Max STD of Gains');
xlabel('t (sec)');
ylabel('STD (unitless)');

subplot(2,2,4);
plot(tObs, max(bwStd)');
title('Max Standard Deviation of Biases');
xlabel('t (sec)');
ylabel('Standard Deviation (rad/s)');

fullFilename = fullfile(figureDir, [filename, '_Gyro']);
savefig(fullFilename);

end