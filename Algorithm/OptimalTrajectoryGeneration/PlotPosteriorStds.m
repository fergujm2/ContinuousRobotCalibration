function PlotPosteriorStds(filename)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);

tObs = dataObj.tObs;
thetaCov = dataObj.thetaCov;

for iii = 1:length(tObs)
    stdTheta(:,iii) = sqrt(diag(thetaCov(:,:,iii)));
    [xStd(:,iii), gStd(:,iii), tauStd(:,iii), alphAStd(:,iii), raStd(:,iii), kaStd(:,iii), baStd(:,iii), alphWStd(:,iii), rwStd(:,iii), kwStd(:,iii), bwStd(:,iii)] = UnpackTheta(stdTheta(:,iii));
end

[~, ~, ~, paramsMm, paramsDeg] = GetRobotCalibInfo();

xStdMm = xStd(paramsMm,:).*1000;
xStdDeg = rad2deg(xStd(paramsDeg,:));

h = figure(1);
h.Color = [1,1,1];

subplot(2,1,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdMm(1:end-3,:)');

grid on;
ylabel('STD (mm)', 'interpreter', 'latex');
title('Robot Length Parameters', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(2,1,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdDeg');

grid on;
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD ($^\circ$)', 'interpreter', 'latex');
title('Robot Angle Parameters', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

h = figure(2);
h.Color = [1,1,1];

subplot(2,3,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, gStd(1:2,:)');

grid on;
title('Gravity Direction', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (m/s$^2$)', 'interpreter', 'latex');
legend('$g_x$', '$g_y$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(2,3,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, tauStd);

grid on;
title('Time Offset', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (sec)', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(2,3,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdMm((end-2):end,:)');

grid on;
title('Sensor Position', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (sec)', 'interpreter', 'latex');
legend('$t_x$', '$t_y$', '$t_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual');
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(2,2,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(raStd'));

grid on;
title('Accel. Orientation', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$r_z$', '$r_y$', '$r_x$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(2,2,4,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(rwStd'));

grid on;
title('Gyro. Orientation', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$r_z$', '$r_y$', '$r_x$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

h = figure(3);
h.Color = [1,1,1];

subplot(3,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(alphAStd'));

grid on;
title('Accel. Misalignments', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$\gamma_{yz}$', '$\gamma_{zy}$', '$\gamma_{zx}$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(3,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(alphWStd'));

grid on;
title('Gyro. Misalignments', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$\gamma_{yz}$', '$\gamma_{zy}$', '$\gamma_{zx}$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(3,2,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, kaStd');

grid on;
title('Accel. Gains', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (unitless)', 'interpreter', 'latex');
legend('$k_x$', '$k_y$', '$k_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(3,2,4,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, kwStd');

grid on;
title('Gyro. Gains', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (unitless)', 'interpreter', 'latex');
legend('$k_x$', '$k_y$', '$k_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(3,2,5,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, baStd');

grid on;
title('Accel. Biases', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (m/s/s)', 'interpreter', 'latex');
legend('$b_x$', '$b_y$', '$b_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

subplot(3,2,6,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, bwStd');

grid on;
title('Gyro. Biases', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (rad/s)', 'interpreter', 'latex');
legend('$b_x$', '$b_y$', '$b_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
xticklabels('manual')
xticks([0, 10, 100, 300]);
xticklabels({'0','10','100','300'});

saveFigurePdf([7.16, 8]);

end

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end