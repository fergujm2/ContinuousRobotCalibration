function RunMonteCarloCalibration(dataDir, dataFilename)

dataFullFilename = fullfile('..', dataDir, 'DataProcessed', dataFilename);
dataObj = load(dataFullFilename);

tRobot = dataObj.tRobot;
tImu = dataObj.tImu;
q = dataObj.q;
z = dataObj.z;

thetaNominal = GetThetaNominal();
thetaTruth = GetThetaTruth();

tSpan = 10;
tTrim = 2;

numCalibrations = 100;

a = tImu(1);
b = tImu(end);

tPartitions = a:(tSpan + tTrim):b;
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

outputFilename = [dataFilename, '_', num2str(tSpan), 's_', num2str(numCalibrations), 'Calib'];
outputFullFilename = fullfile('..', dataDir, 'OutputCalibrations', outputFilename);
save(outputFullFilename, 'thetaStar', 'thetaStarCov', 'thetaNominal', 'thetaTruth');

end