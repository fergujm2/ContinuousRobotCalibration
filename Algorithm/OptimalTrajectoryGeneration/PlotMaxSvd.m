function PlotMaxSvd()

fullFilenameOptimal = fullfile('Output', 'BSpline_d3_step5_300s_Analyzed');
fullFilenameRandom = fullfile('Output', 'BSplineRandom_d3_300s_Analyzed');

dataObjOptimal = load(fullFilenameOptimal);
dataObjRandom = load(fullFilenameRandom);

tObs = dataObjOptimal.tObs;
thetaCovOptimal = dataObjOptimal.thetaCov;
thetaCovRandom = dataObjRandom.thetaCov;

for iii = 1:length(tObs)
    maxSvdOptimal(iii) = max(svd(thetaCovOptimal(:,:,iii)));
    maxSvdRandom(iii) = max(svd(thetaCovRandom(:,:,iii)));
end

h = figure(4);
clf;
h.Color = [1,1,1];
hold on;
set(gca,'XScale', 'log', 'YScale', 'log');

plot(tObs, maxSvdOptimal);
plot(tObs, maxSvdRandom);

xlabel('Trajectory Length (sec)', 'interpreter', 'latex');
ylabel('max svd($\Sigma_\pi$) (-)', 'interpreter', 'latex');
legend('Optimal Trajectory', 'Random Trajectory', 'interpreter', 'latex');
yticklabels('manual');
yticks([1e-4, 1e-2, 1e0]);
yticklabels({'10^{-4}','10^{-2}','10^{0}'});

ax = gca;
ax.FontSize = 8;
grid on;

saveFigurePdf([3.5, 1.75]);

end

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end