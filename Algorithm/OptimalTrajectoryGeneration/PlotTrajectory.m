function PlotTrajectory(filename)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);
y = dataObj.y;
C = dataObj.C;
d = dataObj.d;
tSpan = dataObj.tSpan;

% Now, we need the end points to be zero
numZeros = 3;
C(:,(end - numZeros + 1):end) = repmat(C(:,1), 1, numZeros);

sampleRate = 300;
numMeas = sampleRate*(tSpan(2) - tSpan(1));

t = linspace(tSpan(1), tSpan(2), numMeas);
q = EvalVectorSpline(y, C, d, t);

h = figure(6);
h.Color = [1,1,1];

plot(t, q);

xlabel('time (sec)', 'Interpreter', 'latex');
ylabel('Joint values, $\boldmath{q}$ ($^{\circ}$)', 'Interpreter', 'latex');
leg = legend('$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', 'Interpreter', 'latex');
leg.ItemTokenSize = [5,18];
leg.Location = 'southeast';
leg.Orientation = 'horizontal';

ax = gca;
ax.FontSize = 8;

saveFigurePdf([3.5, 2]);


t = 150:(1/sampleRate):180;
q = EvalVectorSpline(y, C, d, t);

h = figure(7);
h.Color = [1,1,1];

plot(t, q);

xlabel('time (sec)', 'Interpreter', 'latex');

ax = gca;
ax.FontSize = 8;

saveFigurePdf([3.5, 2]);

end

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end