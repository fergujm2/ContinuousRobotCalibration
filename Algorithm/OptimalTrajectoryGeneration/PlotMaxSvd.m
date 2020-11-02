function PlotMaxSvd()

fullFilenameOptimal = fullfile('Output', 'BSpline_d3_step5_300s_Rot_AnalyzedRot');
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

h = figure(8);
clf;
h.Color = [1,1,1];
hold on;
set(gca,'XScale', 'log', 'YScale', 'log');

plot(tObs, sqrt(maxSvdOptimal));
plot(tObs, sqrt(maxSvdRandom));

grid on;
xlabel('time (sec)', 'interpreter', 'latex');
ylabel('Observability Measure', 'interpreter', 'latex');
legend('Optimal Trajectory', 'Random Trajectory', 'interpreter', 'latex');

ax = gca;
ax.FontSize = 8;

saveFigurePdf([3.5, 2.5]);

end

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end