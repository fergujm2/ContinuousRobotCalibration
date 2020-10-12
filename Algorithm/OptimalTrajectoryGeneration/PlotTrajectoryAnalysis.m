function PlotTrajectoryAnalysis(filename, lineSpec)

fullFilename = fullfile('Output', filename);
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

h = plot(tObs, xStdMm', lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));

xlabel('t (sec)');
ylabel('STD (mm)');
title('Posterior STD of Robot Length Parameters');

subplot(1,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, xStdDeg', lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
ylabel('STD (deg)');
title('Posterior STD of Robot Angle Parameters');

figure(2);

subplot(1,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, gStd(1:2,:)', lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Gravity');
xlabel('t (sec)');
ylabel('STD (m/s/s)');

subplot(1,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, tauStd, lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Time Offset');
xlabel('t (sec)');
ylabel('STD (sec)');

figure(3);

subplot(2,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, rad2deg(alphAStd'), lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Axis Misalignments');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, rad2deg(raStd'), lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Orientation');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,3,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, kaStd', lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Gains');
xlabel('t (sec)');
ylabel('STD (unitless)');

subplot(2,2,4,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, baStd', lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Biases');
xlabel('t (sec)');
ylabel('STD (m/s/s)');

figure(4);

subplot(2,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, rad2deg(alphWStd'), lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Axis Misalignments');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, rad2deg(rwStd'), lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Orientation');
xlabel('t (sec)');
ylabel('STD (deg)');

subplot(2,2,3,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, kwStd', lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Gains');
xlabel('t (sec)');
ylabel('STD (unitless)');

subplot(2,2,4,'XScale', 'log', 'YScale', 'log');
hold on;

h = plot(tObs, bwStd', lineSpec);
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
title('STD of Biases');
xlabel('t (sec)');
ylabel('Standard Deviation (rad/s)');