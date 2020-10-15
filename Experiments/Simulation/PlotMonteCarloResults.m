clear;

dataFilename = 'BSpline_d3_step5_300s_Rot_100Calib';

dataFullFilename = fullfile('OutputCalibrations', dataFilename);

outputObj = load(dataFullFilename);

thetaStar = outputObj.thetaStar;
thetaTruth = outputObj.thetaTruth;

[xTruth, gTruth, tauTruth, alphATruth, raTruth, kaTruth, baTruth, alphWTruth, rwTruth, kwTruth, bwTruth] = UnpackTheta(thetaTruth);

numCalibrations = size(thetaStar, 2);
numBins = ceil(numCalibrations/10);
% numBins = 2;

[xStar, gStar, tauStar, alphAStar, raStar, kaStar, baStar, alphWStar, rwStar, kwStar, bwStar] = UnpackTheta(thetaStar(:,1));

% Unpack all of the thetaStars
for iii = 1:numCalibrations
    [xStar(:,iii), gStar(:,iii), tauStar(:,iii), alphAStar(:,iii), raStar(:,iii), kaStar(:,iii), baStar(:,iii), alphWStar(:,iii), rwStar(:,iii), kwStar(:,iii), bwStar(:,iii)] = UnpackTheta(thetaStar(:,iii));
end

%% Plot Robot Parameters
degScale = [-.25, .25];
mmScale = [-5, 5];
mssScale = [-.025, .025];
kScale = [-3,3];

[calibBools, numParams, numParamsTotal] = GetRobotCalibInfo();

paramNames = cell(1, numParamsTotal);

rOrT = repmat([repmat({'t_'}, 1, 3), repmat({'r_'}, 1, 3)], 1, numParamsTotal/6);
subscriptsT = reshape(repmat(1:(numParamsTotal/6), 6, 1), 1, numParamsTotal);
subscripts3 = repmat({',x', ',y', ',z', ',y', ',z', ',x'}, 1, numParamsTotal/6);

for iii = 1:numParamsTotal
    paramNames{iii} = ['e_{', rOrT{iii}, '{', num2str(subscriptsT(iii)), subscripts3{iii}, '}}'];
end

paramNames = paramNames(calibBools);

numFigCols = 3;
numFigRows = ceil((numParams - 3)/numFigCols);

h = figure(1);
h.Color = [1,1,1];

for iii = 1:(numParams - 3)
    subplot(numFigRows,numFigCols,iii);
    hold on;
    
    paramName = paramNames(iii);
    if contains(paramName, 't')
        units = '(mm)';
        mult = 1000;
        xlim(mmScale);
    else
        units = ' (^{\circ})';
        mult = 180/pi;
        xlim(degScale);
    end
    
    a = (xStar(iii,:) - xTruth(iii))*mult;
    
    h = histogram(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', paramNames{iii}, units, '$'], 'Interpreter', 'Latex');
    ax = gca;
    ax.FontSize = 8;
end

saveFigurePdf([3.45, 3.45]);

%% Plot Other Extrinsic Parameters (g, tau, sensor posistion)

gNames = {'g_{x}', 'g_{y}'};

h = figure(2);
h.Color = [1,1,1];

for iii = 1:length(gNames)
    subplot(2,3,iii);
    hold on;
    
    a = gStar(iii,:) - gTruth(iii);
    
    h = histogram(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', gNames{iii}, '$ (m/s', '$^2$)'], 'Interpreter', 'Latex');
    xlim(mssScale);
    ax = gca;
    ax.FontSize = 8;
    ax.XAxis.Exponent = 0;
end

subplot(2,3,3);
hold on;

h = histogram((tauStar - tauTruth)*1000, numBins);
h(1).FaceAlpha = 0.25;

xlabel('$e_{\tau}$ (ms)', 'Interpreter', 'Latex');
xlim([-.25, .25]);
ax = gca;
ax.FontSize = 8;

for iii = 1:3
    subplot(2,3,3 + iii);
    hold on;
    
    paramName = paramNames(numParams - 3 + iii);
    
    a = (xStar(iii,:) - xTruth(iii))*1000;
    
    h = histogram(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', paramNames{iii}, '$', '(mm)'], 'Interpreter', 'Latex');
    xlim(mmScale);
    ax = gca;
    ax.FontSize = 8;
end

saveFigurePdf([3.45, 2]);

%% Plot Accelerometer Intrinsics

alphNames = {'\gamma_{yz}', '\gamma_{zy}', '\gamma_{zx}'};
rNames = {'r_z', 'r_y', 'r_x'};
kNames = {'k_x', 'k_y', 'k_z'};
bNames = {'b_x', 'b_y', 'b_z'};

h = figure(3);
h.Color = [1,1,1];

for iii = 1:length(alphNames)
    subplot(4,3,iii);
    hold on;
    
    a = alphAStar(iii,:) - alphATruth(iii);
    
    h = histogram(rad2deg(a), numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', alphNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
    xlim(degScale);
    ax = gca;
    ax.FontSize = 8;
end

for iii = 1:length(rNames)
    subplot(4,3,iii + 3);
    hold on;
    
    a = raStar(iii,:) - raTruth(iii);
    
    h = histogram(rad2deg(a), numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', rNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
    xlim(degScale);
    ax = gca;
    ax.FontSize = 8;
end

for iii = 1:length(kNames)
    subplot(4,3,iii + 6);
    hold on;
    
    a = kaStar(iii,:) - kaTruth(iii);
    
    h = histogram(1000*a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', kNames{iii}, '\times 1000$'], 'Interpreter', 'Latex');
    xlim(kScale);
    ax = gca;
    ax.FontSize = 8;
end

for iii = 1:length(bNames)
    subplot(4,3,iii + 9);
    hold on;
    
    a = baStar(iii,:) - baTruth(iii);
    
    h = histogram(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', bNames{iii}, '$ (m/s', '$^2$)'], 'Interpreter', 'Latex');
    xlim(mssScale);
    ax = gca;
    ax.FontSize = 8;
end

saveFigurePdf([3.45, 3.45]);

%% Plot Gyroscope Intrinsics

h = figure(4);
h.Color = [1,1,1];

for iii = 1:length(alphNames)
    subplot(4,3,iii);
    hold on;
    
    a = alphWStar(iii,:) - alphWTruth(iii);
    
    h = histogram(rad2deg(a), numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', alphNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
    xlim(degScale);
    ax = gca;
    ax.FontSize = 8;
end

for iii = 1:length(rNames)
    subplot(4,3,iii + 3);
    hold on;
    
    a = rwStar(iii,:) - rwTruth(iii);
    
    h = histogram(rad2deg(a), numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', rNames{iii}, ' (^{\circ})', '$'], 'Interpreter', 'Latex');
    xlim(degScale);
    ax = gca;
    ax.FontSize = 8;
end

for iii = 1:length(kNames)
    subplot(4,3,iii + 6);
    hold on;
    
    a = kwStar(iii,:) - kwTruth(iii);
    
    h = histogram(1000*a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', kNames{iii}, '\times 1000$'], 'Interpreter', 'Latex');
    xlim(kScale);
    ax = gca;
    ax.FontSize = 8;
end

for iii = 1:length(bNames)
    subplot(4,3,iii + 9);
    hold on;
    
    a = bwStar(iii,:) - bwTruth(iii);
    
    h = histogram(rad2deg(a), numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', bNames{iii}, ' (^{\circ}/s)', '$'], 'Interpreter', 'Latex');
    xlim([-.025, .025]);
    ax = gca;
    ax.FontSize = 8;
end

saveFigurePdf([3.45, 3.45]);

function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end