clear;

dataDir = fullfile('..', 'Simulation');
dataFilename = 'SimulatedMeasurements_Full_60s.mat';

% dataDir = fullfile('..', '20200730_FirstDataSet');
% dataFilename = 'ImuCalibrationDataOffset_10s_20200730_151522.mat';
% dataFilename = 'ImuCalibrationDataOffset_25s_20200730_152436.mat';
% dataFilename = 'ImuCalibrationDataFull_10s_20200730_145802.mat';

dataFullFilename = fullfile(dataDir, 'OutputCalibrations', dataFilename);

outputObj = load(dataFullFilename);

thetaStar = outputObj.thetaStar;
thetaTruth = outputObj.thetaTruth;

[xTruth, gTruth, tauTruth, sTruth, bTruth] = UnpackTheta(thetaTruth);

numCalibrations = size(thetaStar, 2);

[xStar, gStar, tauStar, sStar, bStar] = UnpackTheta(thetaStar(:,1));

% Unpack all of the thetaStars
for iii = 1:numCalibrations
    [xStar(:,iii), gStar(:,iii), tauStar(:,iii), sStar(:,iii), bStar(:,iii)] = UnpackTheta(thetaStar(:,iii));
end

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
    
    histfit(a);
    xlabel(['$', paramNames{iii}, '$', units], 'Interpreter', 'Latex');
    grid on;
end

gNames = {'g_{x}', 'g_{y}', 'g_{z}'};

figure(3);

for iii = 1:length(gNames)
    subplot(4,1,iii);
    hold on;
    
    a = gStar(iii,:) - gTruth(iii);
    
    histfit(a);
    xlabel(['$', gNames{iii}, '$', ' m/s/s'], 'Interpreter', 'Latex');
    grid on;
end

subplot(4,1,4);
hold on;

histfit(tauStar - tauTruth);
xlabel('$e_{\tau}$ (s)', 'Interpreter', 'Latex');
grid on;


figure(4);

subplot(4, 2, 1);
hold on;

histfit(sStar(1,:) - sTruth(1));
xlabel('$e_{s_{1}}$', 'Interpreter', 'Latex');

subplot(4, 2, 2);
hold on;

histfit(sStar(2,:) - sTruth(2));
xlabel('$e_{s_{2}}$', 'Interpreter', 'Latex');

for iii = 1:6
    subplot(4, 2, 2 + iii);
    hold on;
    
    histfit(bStar(iii,:) - bTruth(iii));
    xlabel(['$e_{b_{', num2str(iii), '}}$'], 'Interpreter', 'Latex');
    grid on;
end




% subplot(5,1,5);
% hold on;
% y = [sStar, sTruth];
% bar(y);
% 
% ngroups = size(y, 1); nbars = size(y, 2);
% groupwidth = min(0.8, nbars/(nbars + 1.5));
% x = (1:ngroups) - groupwidth/2 + (2*1-1) * groupwidth / (2*nbars);
% errorbar(x, y(:,1), sStarStd, '.k');
% 
% title('Data Scale Factors');
% 
% drawnow();
