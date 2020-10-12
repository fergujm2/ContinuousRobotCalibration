clear;

dataDir = fullfile('..', 'Simulation');
dataFilename = 'OptimalTrajectory_Full_10n_120s_50Calib.mat';

% dataDir = fullfile('..', '20200903_TestingNewSetup');
% dataFilename = '5n10s.mat';

dataFullFilename = fullfile(dataDir, 'OutputCalibrations', dataFilename);

outputObj = load(dataFullFilename);

thetaStar = outputObj.thetaStar;
thetaTruth = outputObj.thetaTruth;
% thetaTruth = zeros(size(GetThetaNominal()));

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
numFigRows = ceil(numParams/numFigCols);

figure(2);

for iii = 1:numParams
    subplot(numFigRows,numFigCols,iii);
    hold on;
    
    paramName = paramNames(iii);
    if contains(paramName, 't')
        units = ' (mm)';
        mult = 1000;
    else
        units = ' (deg)';
        mult = 180/pi;
    end
    
    a = (xStar(iii,:) - xTruth(iii))*mult;
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', paramNames{iii}, '$', units], 'Interpreter', 'Latex');
    grid on;
end

%% Plot Other Extrinsic Parameters (g, tau)

gNames = {'g_{x}', 'g_{y}', 'g_{z}'};

figure(3);

for iii = 1:length(gNames)
    subplot(4,1,iii);
    hold on;
    
    a = gStar(iii,:) - gTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', gNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

subplot(4,1,4);
hold on;

h = histfit(tauStar - tauTruth, numBins);
h(1).FaceAlpha = 0.25;

xlabel('$e_{\tau}$ (s)', 'Interpreter', 'Latex');
grid on;

%% Plot Accelerometer Intrinsics

alphNames = {'\alpha_{yz}', '\alpha_{zy}', '\alpha_{zx}'};
rNames = {'\theta_{z}', '\theta_{y}', '\theta_{x}'};
kNames = {'k_x', 'k_y', 'k_z'};
bNames = {'b_x', 'b_y', 'b_z'};

figure(4);

for iii = 1:length(alphNames)
    subplot(6,2,iii);
    hold on;
    
    a = alphAStar(iii,:) - alphATruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', alphNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

for iii = 1:length(rNames)
    subplot(6,2,iii + 3);
    hold on;
    
    a = raStar(iii,:) - raTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', rNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

for iii = 1:length(kNames)
    subplot(6,2,iii + 6);
    hold on;
    
    a = kaStar(iii,:) - kaTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', kNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

for iii = 1:length(bNames)
    subplot(6,2,iii + 9);
    hold on;
    
    a = baStar(iii,:) - baTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', bNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

%% Plot Gyroscope Intrinsics

figure(5);

for iii = 1:length(alphNames)
    subplot(6,2,iii);
    hold on;
    
    a = alphWStar(iii,:) - alphWTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', alphNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

for iii = 1:length(rNames)
    subplot(6,2,iii + 3);
    hold on;
    
    a = rwStar(iii,:) - rwTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', rNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

for iii = 1:length(kNames)
    subplot(6,2,iii + 6);
    hold on;
    
    a = kwStar(iii,:) - kwTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', kNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

for iii = 1:length(bNames)
    subplot(6,2,iii + 9);
    hold on;
    
    a = bwStar(iii,:) - bwTruth(iii);
    
    h = histfit(a, numBins);
    h(1).FaceAlpha = 0.25;
    
    xlabel(['$', bNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end
