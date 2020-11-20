function RunCalibration(dataDir, filename, tSpan)

if nargin == 2
    tSpan = [0, inf];
end

fullFilename = fullfile('..', dataDir, 'DataProcessed', filename);
dataObj = load(fullFilename);

tRobot = dataObj.tRobot;
q = dataObj.q;
tImu = dataObj.tImu;
z = dataObj.z;

rowsToKeep = and(tRobot > tSpan(1), tRobot < tSpan(2));
tRobot = tRobot(rowsToKeep);
q = q(rowsToKeep,:);

rowsToKeep = and(tImu > tSpan(1), tImu < tSpan(2));
tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

[thetaStar, CStar, y, d, thetaStarCov] = ComputeImuCalibration(tRobot', q, tImu', z);

outputFilename = [filename, '_', num2str(tSpan(1)), 'To', num2str(tSpan(2))];
outputFullFilename = fullfile('..', dataDir, 'OutputCalibrations', outputFilename);
save(outputFullFilename, 'thetaStar', 'thetaStarCov', 'y', 'd', 'CStar');
end