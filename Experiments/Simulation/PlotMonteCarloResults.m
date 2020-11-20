clear;
clf;

trajectoryFullFilename = fullfile('..', '..', 'Algorithm', 'OptimalTrajectoryGeneration', 'Output', 'BSpline_d3_step5_300s');
dataObj = load(trajectoryFullFilename);
yTruth = dataObj.y;
CTruth = dataObj.C;
dTruth = dataObj.d;
qTruth = @(t) EvalVectorSpline(yTruth, CTruth, dTruth, t);

trajectoryAnalysisFilename = [trajectoryFullFilename, '_Analyzed'];
dataObj = load(trajectoryAnalysisFilename);
thetaStdPred = sqrt(diag(dataObj.thetaCov(:,:,end)));
[xStdPred, gStdPred, tauStdPred, alphAStdPred, raStdPred, kaStdPred, baStdPred, alphWStdPred, rwStdPred, kwStdPred, bwStdPred] = UnpackTheta(thetaStdPred);

dataFilenames = dir(fullfile('OutputCalibrations', '*.mat'));
numCalibrations = length(dataFilenames);

for iii = 1:numCalibrations
    outputObj = load(fullfile('OutputCalibrations', dataFilenames(iii).name));
    
    thetaStar(:,iii) = outputObj.thetaStar';
    thetaTruth(:,iii) = outputObj.thetaTruth';
    
    [xStar(:,iii), gStar(:,iii), tauStar(:,iii), alphAStar(:,iii), raStar(:,iii), kaStar(:,iii), baStar(:,iii), alphWStar(:,iii), rwStar(:,iii), kwStar(:,iii), bwStar(:,iii)] = UnpackTheta(outputObj.thetaStar);
    [xTruth(:,iii), gTruth(:,iii), tauTruth(:,iii), alphATruth(:,iii), raTruth(:,iii), kaTruth(:,iii), baTruth(:,iii), alphWTruth(:,iii), rwTruth(:,iii), kwTruth(:,iii), bwTruth(:,iii)] = UnpackTheta(outputObj.thetaTruth);

    qStar{iii} = @(t) EvalVectorSpline(outputObj.y, outputObj.CStar, outputObj.d, t);
end

thetaStd = sqrt(diag(cov(thetaTruth' - thetaStar')));

%% Plot Robot Parameters
degScale = [-.25, .25];
mmScale = [-5, 5];
mssScale = [-.025, .025];
kScale = [-3,3];

[calibBools, numParams, numParamsTotal, paramsMm] = GetRobotCalibInfo();

paramNames = cell(1, numParamsTotal);

kkk = 1;
for iii = 0:6
    for jjj = 1:6
        paramNames{kkk} = ['\epsilon_{', num2str(iii), '_', num2str(jjj), '}'];
        kkk = kkk + 1;
    end
end

paramNames = paramNames(calibBools);

h = figure(1);
h.Color = [1,1,1];

plotInd = 1;

for iii = 1:(numParams - 3)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    
    paramName = paramNames(iii);
    if contains(paramName, '_1') || contains(paramName, '_2') || contains(paramName, '_3')
        units = '(mm)';
        mult = 1000;
        xlim(mmScale);
    else
        units = ' ($^{\circ}$)';
        mult = 180/pi;
        xlim(degScale);
    end
    
    a = (xStar(iii,:) - xTruth(iii,:))*mult;
    histPlot(a, xStdPred(iii)*mult);
    
    xlabel(['$', paramNames{iii}, '$', units], 'Interpreter', 'Latex');
end

%% Plot Other Extrinsic Parameters (g, tau, sensor posistion)

gNames = {'g_{x}', 'g_{y}'};

for iii = 1:length(gNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(mssScale); 
    
    a = gStar(iii,:) - gTruth(iii,:);
    histPlot(a, gStdPred(iii));

    xlabel(['$', gNames{iii}, '$ (m/s', '$^2$)'], 'Interpreter', 'Latex');
end

subplot(6,8,plotInd);
plotInd = plotInd + 1;
hold on;

histPlot((tauStar - tauTruth)*1000, 1000*tauStdPred);

xlabel('$\tau$ (ms)', 'Interpreter', 'Latex');

for iii = 1:3
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(mmScale);
    
    paramName = paramNames{numParams - 3 + iii};
    
    a = (xStar(numParams - 3 + iii,:) - xTruth(numParams - 3 + iii,:))*1000;
    histPlot(a, 1000*xStdPred(numParams - 3 + iii));
    
    xlabel(['$', paramName, '$', '(mm)'], 'Interpreter', 'Latex');
end

%% Plot Accelerometer Intrinsics

alphNames = {'\gamma_{a_{yz}}', '\gamma_{a_{zy}}', '\gamma_{a_{zx}}'};
rNames = {'r_{a_z}', 'r_{a_y}', 'r_{a_x}'};
kNames = {'k_{a_x}', 'k_{a_y}', 'k_{a_z}'};
bNames = {'b_{a_x}', 'b_{a_y}', 'b_{a_z}'};

for iii = 1:length(alphNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(degScale);
    
    a = alphAStar(iii,:) - alphATruth(iii,:);
    histPlot(rad2deg(a), rad2deg(alphAStdPred(iii)));
    
    xlabel(['$', alphNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
end

for iii = 1:length(rNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(degScale);
    
    a = raStar(iii,:) - raTruth(iii,:);
    histPlot(rad2deg(a), rad2deg(raStdPred(iii)));
   
    xlabel(['$', rNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
end

for iii = 1:length(kNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(kScale);
    
    a = kaStar(iii,:) - kaTruth(iii,:);
    histPlot(1000*a, 1000*kaStdPred(iii));
    
    xlabel(['$', kNames{iii}, '\times 1000$'], 'Interpreter', 'Latex');
end

for iii = 1:length(bNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(mssScale);
    
    a = baStar(iii,:) - baTruth(iii,:);
    histPlot(a, baStdPred(iii));
    
    xlabel(['$', bNames{iii}, '$ (m/s', '$^2$)'], 'Interpreter', 'Latex');
end

%% Plot Gyroscope Intrinsics

alphNames = {'\gamma_{\omega_{yz}}', '\gamma_{\omega_{zy}}', '\gamma_{\omega_{zx}}'};
rNames = {'r_{\omega_z}', 'r_{\omega_y}', 'r_{\omega_x}'};
kNames = {'k_{\omega_x}', 'k_{\omega_y}', 'k_{\omega_z}'};
bNames = {'b_{\omega_x}', 'b_{\omega_y}', 'b_{\omega_z}'};

for iii = 1:length(alphNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(degScale);
    
    a = alphWStar(iii,:) - alphWTruth(iii,:);
    histPlot(rad2deg(a), rad2deg(alphWStdPred(iii)));
    
    xlabel(['$', alphNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
end

for iii = 1:length(rNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(degScale);
    
    a = rwStar(iii,:) - rwTruth(iii,:);
    histPlot(rad2deg(a), rad2deg(rwStdPred(iii)));
    
    xlabel(['$', rNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
end

for iii = 1:length(kNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim(kScale);
    
    a = kwStar(iii,:) - kwTruth(iii,:);
    histPlot(1000*a, 1000*kwStdPred(iii));
    
    xlabel(['$', kNames{iii}, '\times 1000$'], 'Interpreter', 'Latex');
end

for iii = 1:length(bNames)
    subplot(6,8,plotInd);
    plotInd = plotInd + 1;
    hold on;
    xlim([-.025, .025]);
    
    a = bwStar(iii,:) - bwTruth(iii,:);
    histPlot(rad2deg(a), rad2deg(bwStdPred(iii)));
    
    xlabel(['$', bNames{iii}, ' (^{\circ}/s)', '$'], 'Interpreter', 'Latex');
end

leg = legend('Histogram','Predicted', 'Interpreter', 'Latex');
leg.ItemTokenSize = [7.5, 30];
saveFigurePdf([9, 5]);


function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end

function histPlot(a, stdPred)
    h = histogram(a, 10);
    h.FaceAlpha = 0.6;
    h.FaceColor = 'b';
    
    a = gca;
    xSpan = a.XLim;
    x = linspace(xSpan(1), xSpan(2), 100);
    gauss = @(x, mu, sig, amp, vo) exp(log(amp) - (((x - mu).^2)/(2*sig.^2))) + vo;
    y = gauss(x, 0, stdPred, max(h.Values), 0);
    plot(x, y, '-r', 'LineWidth', 1);
    
    a.FontSize = 8;
end