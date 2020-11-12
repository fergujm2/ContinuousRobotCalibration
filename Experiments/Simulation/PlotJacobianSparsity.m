clear;

trajectoryFilename = 'BSplineRandom_d3_15s';
trajectoryFullFilename = fullfile('..', '..', 'Algorithm', 'OptimalTrajectoryGeneration', 'Output', trajectoryFilename);

n = 15;
tRobot = linspace(1, 15, n);
tImu = tRobot;

dataObj = load(trajectoryFullFilename);
y = dataObj.y;
C = dataObj.C;
d = dataObj.d;

JSparsity = GetJacobianSparsity(tRobot, tImu, y, d, C);

figure(5);

% imshow(not(full(JSparsity)));
spy(JSparsity, 'k', 2);

h = gca;
h.Visible = 'On';
h.FontSize = 8;

xlabel('Jacobian Columns', 'Interpreter', 'Latex');
ylabel('Jacobian Rows', 'Interpreter', 'Latex');

saveFigurePdf([3.45, 2]);

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end
