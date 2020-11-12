function PlotPosteriorStds()

filename = 'BSpline_d3_step5_300s_Analyzed';

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
clf;
h.Color = [1,1,1];

subplot(1,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdMm(1:end-3,:)');

ylabel('STD (mm)', 'interpreter', 'latex');
title('Robot Length Parameters', 'interpreter', 'latex');
setupAx();

subplot(1,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdDeg');

ylabel('STD ($^\circ$)', 'interpreter', 'latex');
title('Robot Angle Parameters', 'interpreter', 'latex');
setupAx();

saveFigurePdf([7, 1.25]);


h = figure(2);
clf;
h.Color = [1,1,1];

subplot(2,3,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, gStd(1:2,:)');

title('Gravity Direction', 'interpreter', 'latex');
ylabel('STD (m/s$^2$)', 'interpreter', 'latex');
legend('$g_x$', '$g_y$', 'interpreter', 'latex');
setupAx();
yticklabels('manual');
yticks([0.01, 0.1]);
yticklabels({'10^{-2}','10^{-1}'});

subplot(2,3,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, tauStd);

title('Time Offset', 'interpreter', 'latex');
ylabel('STD (sec)', 'interpreter', 'latex');
setupAx();

subplot(2,3,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdMm((end-2):end,:)');

title('Sensor Position', 'interpreter', 'latex');
ylabel('STD (mm)', 'interpreter', 'latex');
legend('$\epsilon_{6_1}$', '$\epsilon_{6_2}$', '$\epsilon_{6_3}$', 'interpreter', 'latex');
setupAx();

subplot(2,2,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(raStd'));

title('Accelerometer Orientation', 'interpreter', 'latex');
ylabel('STD ($^\circ$)', 'interpreter', 'latex');
legend('$r_{a_z}$', '$r_{a_y}$', '$r_{a_x}$', 'interpreter', 'latex');
setupAx();
yticklabels('manual');
yticks([0.1, 1]);
yticklabels({'10^{-1}','10^{0}'});

subplot(2,2,4,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(rwStd'));

title('Gyroscope Orientation', 'interpreter', 'latex');
ylabel('STD ($^\circ$)', 'interpreter', 'latex');
legend('$r_{\omega_z}$', '$r_{\omega_y}$', '$r_{\omega_x}$', 'interpreter', 'latex');
setupAx();

saveFigurePdf([9, 2.25]);


h = figure(3);
clf;
h.Color = [1,1,1];

subplot(2,3,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(alphAStd'));

title('Accelerometer Misalignments', 'interpreter', 'latex');
ylabel('STD ($^\circ$)', 'interpreter', 'latex');
legend('$\gamma_{a_{yz}}$', '$\gamma_{a_{zy}}$', '$\gamma_{a_{zx}}$', 'interpreter', 'latex');
setupAx();
yticklabels('manual');
yticks([0.1, 1]);
yticklabels({'10^{-1}','10^{0}'});

subplot(2,3,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, kaStd');

title('Accelerometer Gains', 'interpreter', 'latex');
ylabel('STD (-)', 'interpreter', 'latex');
legend('$k_{a_x}$', '$k_{a_y}$', '$k_{a_z}$', 'interpreter', 'latex');
setupAx();
yticklabels('manual');
yticks([0.001, 0.01]);
yticklabels({'10^{-3}','10^{-2}'});

subplot(2,3,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, baStd');

title('Accelerometer Biases', 'interpreter', 'latex');
ylabel('STD (m/s$^2$)', 'interpreter', 'latex');
legend('$b_{a_x}$', '$b_{a_y}$', '$b_{a_z}$', 'interpreter', 'latex');
setupAx();
yticklabels('manual');
yticks([0.01, 0.1]);
yticklabels({'10^{-2}','10^{-1}'});

subplot(2,3,4,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(alphWStd'));

title('Gyroscope Misalignments', 'interpreter', 'latex');
ylabel('STD ($^\circ$)', 'interpreter', 'latex');
legend('$\gamma_{\omega_{yz}}$', '$\gamma_{\omega_{zy}}$', '$\gamma_{\omega_{zx}}$', 'interpreter', 'latex');
setupAx();

subplot(2,3,5,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, kwStd');

title('Gyroscope Gains', 'interpreter', 'latex');
ylabel('STD (-)', 'interpreter', 'latex');
legend('$k_{\omega_x}$', '$k_{\omega_y}$', '$k_{\omega_z}$', 'interpreter', 'latex');
setupAx();

subplot(2,3,6,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(bwStd'));

title('Gyroscope Biases', 'interpreter', 'latex');
ylabel('STD ($^\circ$/s)', 'interpreter', 'latex');
legend('$b_{\omega_x}$', '$b_{\omega_y}$', '$b_{\omega_z}$', 'interpreter', 'latex');
setupAx();

saveFigurePdf([9, 2.25]);

end

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end

function setupAx()
    ax = gca;
    ax.FontSize = 8;
    xticklabels('manual');
    xticks([0, 10, 100, 300]);
    xticklabels({'0','10','100','300'});
    xlabel('time (sec)', 'interpreter', 'latex');
    
    if ~isempty(ax.Legend)
        ax.Legend.ItemTokenSize = [7.5,18];
    end
end