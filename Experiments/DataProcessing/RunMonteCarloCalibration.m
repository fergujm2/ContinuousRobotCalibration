function RunMonteCarloCalibration()

% dataDir = fullfile('..', '20200730_FirstDataSet');

% dataFilename = 'ImuCalibrationDataOffset_10s_20200730_151522.mat';
% dataFilename = 'ImuCalibrationDataOffset_25s_20200730_152436.mat';
% dataFilename = 'ImuCalibrationDataFull_10s_20200730_145802.mat';
% dataFilename = 'ImuCalibrationDataFull_25s_20200730_153238.mat';

dataDir = fullfile('..', 'Simulation');
dataFilename = 'SimulatedMeasurements_Full.mat';

dataFullFilename = fullfile(dataDir, 'DataProcessed', dataFilename);
outputFilename = fullfile(dataDir, 'OutputCalibrations', dataFilename);

dataObj = load(dataFullFilename);

tRobot = dataObj.tRobot;
tImu = dataObj.tImu;
q = dataObj.q;
z = dataObj.z;

thetaNominal = GetThetaNominal();
thetaTruth = GetThetaTruth();

tSpan = 60 + 2;
numCalibrations = 100;

a = tImu(1);
b = tImu(end);

tPartitions = a:tSpan:b;
numIntervals = length(tPartitions) - 1;

if numCalibrations > numIntervals
    error('Not enough data for %d calibrations', numCalibrations);
end

thetaStar = nan(length(thetaNominal), numCalibrations);
thetaStarCov = nan(length(thetaNominal),length(thetaNominal),numCalibrations);

for iii = 1:numCalibrations
    ai = tPartitions(iii);
    bi = tPartitions(iii+1);
    
    indImu = and(tImu > ai, tImu < bi);
    tImui = tImu(indImu);
    zi = z(indImu,:);
    
    indRobot = and(tRobot > ai, tRobot < bi);
    tRoboti = tRobot(indRobot);
    qi = q(indRobot,:);

    [thetaStar(:,iii), thetaStarCov(:,:,iii)] = ComputeImuCalibration(tRoboti, qi, tImui, zi, thetaNominal);
    
    fprintf('\nCompleted %d of %d calibrations.\n', iii, numCalibrations);
end

save(outputFilename, 'thetaStar', 'thetaStarCov', 'thetaNominal', 'thetaTruth');

end