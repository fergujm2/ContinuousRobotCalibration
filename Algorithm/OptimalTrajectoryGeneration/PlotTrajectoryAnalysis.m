function PlotTrajectoryAnalysis(filename)

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

if size(xStdMm,1) > 3
    h = figure(1);
    h.Color = [1,1,1];
    hold on;
    set(gca,'XScale', 'log', 'YScale', 'log');
    
    h = plot(tObs, xStdMm(1:end-3,:)');
    set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)));
    
    % Need to fit these curves to a/sqrt(n)
    t0 = 150;
    tFit = tObs(tObs > t0);
    xFit = xStdMm(1:end-3, tObs > t0);
    
    f = @(a, n) a./sqrt(n);
    obj = @(a) f(a, tFit) - xFit;

    a0 = ones(size(xFit, 1), 1);
    a = lsqnonlin(obj, a0);
    
    tExtrap = logspace(log10(max(tFit)), 5, 1000);
    plot(tExtrap, f(a, tExtrap), '--');
    
    grid on;
    xlabel('t (sec)', 'interpreter', 'latex');
    ylabel('STD (mm)', 'interpreter', 'latex');
    title('Robot Length Parameters', 'interpreter', 'latex');
    ax = gca;
    ax.FontSize = 8;
    
    saveFigurePdf([3.45, 3]);
end

h = figure(2);
h.Color = [1,1,1];
hold on;
set(gca,'XScale', 'log', 'YScale', 'log');

plot(tObs, xStdDeg');

grid on;
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
title('Robot Angle Parameters', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;
    
saveFigurePdf([3.45, 3]);

h = figure(3);
h.Color = [1,1,1];

subplot(2,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, gStd(1:2,:)');

grid on;
title('Gravity Direction', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (m/s/s)', 'interpreter', 'latex');
legend('$g_x$', '$g_y$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, tauStd);

grid on;
title('Time Offset', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (sec)', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,1,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, xStdMm((end-2):end,:)');

grid on;
title('Sensor Position', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (sec)', 'interpreter', 'latex');
legend('$t_x$', '$t_y$', '$t_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

saveFigurePdf([3.45, 3]);

h = figure(4);
h.Color = [1,1,1];

subplot(2,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(alphAStd'));

grid on;
title('Axis Misalignments', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$\gamma_{yz}$', '$\gamma_{zy}$', '$\gamma_{zx}$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(raStd'));

grid on;
title('Sensor Orientation', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$r_z$', '$r_y$', '$r_x$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,2,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, kaStd');

grid on;
title('Sensor Gains', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (unitless)', 'interpreter', 'latex');
legend('$k_x$', '$k_y$', '$k_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,2,4,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, baStd');

grid on;
title('Sensor Biases', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (m/s/s)', 'interpreter', 'latex');
legend('$b_x$', '$b_y$', '$b_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

saveFigurePdf([3.45, 3]);

h = figure(5);
h.Color = [1,1,1];

subplot(2,2,1,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(alphWStd'));

grid on;
title('Axis Misalignments', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$\gamma_{yz}$', '$\gamma_{zy}$', '$\gamma_{zx}$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,2,2,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, rad2deg(rwStd'));

grid on;
title('Sensor Orientation', 'interpreter', 'latex');
ylabel('STD (deg)', 'interpreter', 'latex');
legend('$r_z$', '$r_y$', '$r_x$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,2,3,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, kwStd');

grid on;
title('Sensor Gains', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (unitless)', 'interpreter', 'latex');
legend('$k_x$', '$k_y$', '$k_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

subplot(2,2,4,'XScale', 'log', 'YScale', 'log');
hold on;

plot(tObs, bwStd');

grid on;
title('Sensor Biases', 'interpreter', 'latex');
xlabel('t (sec)', 'interpreter', 'latex');
ylabel('STD (rad/s)', 'interpreter', 'latex');
legend('$b_x$', '$b_y$', '$b_z$', 'interpreter', 'latex');
ax = gca;
ax.FontSize = 8;

saveFigurePdf([3.45, 3]);

end

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end