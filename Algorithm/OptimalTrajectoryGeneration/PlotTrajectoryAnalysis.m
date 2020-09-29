function PlotTrajectoryAnalysis(filename)

fullFilename = fullfile('Output', [filename, '_Analyzed']);
dataObj = load(fullFilename);

tObs = dataObj.tObs;
thetaCov = dataObj.thetaCov;
 
for iii = 1:length(tObs)
    stdTheta(:,iii) = sqrt(diag(thetaCov(:,:,iii)));
    [xStd(:,iii), gStd(:,iii), tauStd(:,iii), alphAStd(:,iii), raStd(:,iii), kaStd(:,iii), baStd(:,iii), alphWStd(:,iii), rwStd(:,iii), kwStd(:,iii), bwStd(:,iii)] = UnpackTheta(stdTheta(:,iii));
end

[calibBools, numParams, numParamsTotal] = GetRobotCalibInfo();

paramsMm = logical(repmat([ones(3,1); zeros(3,1)], numParamsTotal/6, 1));
paramsMm = paramsMm(calibBools);
paramsDeg = not(paramsMm);

xStdMm = xStd(paramsMm,:).*1000;
xStdDeg = rad2deg(xStd(paramsDeg,:));

figure(1);

subplot(1,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdMm');
xlabel('t (sec)');
ylabel('STD (mm)');
title('Posterior STD of Robot Length Parameters');

subplot(1,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdDeg');
ylabel('STD (deg)');
title('Posterior STD of Robot Angle Parameters');

figure(2);

subplot(1,2,1);
hold on;

plot(tObs, gStd(1:2,:)');
title('STD of Gravity');
xlabel('t (sec)');
ylabel('STD (m/s/s)');

subplot(1,2,2);
hold on;

plot(tObs, tauStd);
title('STD of Time Offset');
xlabel('t (sec)');
ylabel('STD (sec)');

figure(3);

subplot(2,2,1);
hold on;

plot(tObs, rad2deg(alphAStd'));
title('STD of Axis Misalignments');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,2);
hold on;

plot(tObs, rad2deg(raStd'));
title('STD of Orientation');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,3);
hold on;

plot(tObs, kaStd');
title('STD of Gains');
xlabel('t (sec)');
ylabel('STD (unitless)');

subplot(2,2,4);
hold on;

plot(tObs, baStd');
title('STD of Biases');
xlabel('t (sec)');
ylabel('STD (m/s/s)');

figure(4);

subplot(2,2,1);
hold on;

plot(tObs, rad2deg(alphWStd'));
title('STD of Axis Misalignments');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,2);
hold on;

plot(tObs, rad2deg(rwStd'));
title('STD of Orientation');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,3);
hold on;

plot(tObs, kwStd');
title('STD of Gains');
xlabel('t (sec)');
ylabel('STD (unitless)');

subplot(2,2,4);
hold on;

plot(tObs, bwStd')
title('STD of Biases');
xlabel('t (sec)');
ylabel('Standard Deviation (rad/s)');