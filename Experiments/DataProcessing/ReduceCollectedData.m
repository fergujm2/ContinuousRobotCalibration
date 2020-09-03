function ReduceCollectedData(testDir, filename)

dataFilenameRaw = fullfile('..', testDir, 'DataRaw', filename);
dataFilenameProcessed = fullfile('..', testDir, 'DataProcessed', filename);

tSpan = [5, 1000];

dataObj = load(dataFilenameRaw);

imuMsgData = dataObj.imuMsgData;
robotMsgData = dataObj.robotMsgData;

tRobot = robotMsgData(:,1);
qDes = robotMsgData(:,2:7);
q = robotMsgData(:,8:end);

tImu = imuMsgData(:,1);
z = imuMsgData(:,2:end);

% Shift timestamps from initial values
tRobot = tRobot - tRobot(1);
tImu = tImu - tImu(1);

% Determine rows to keep based on timespan of interest
rowsToKeep = tImu >= tSpan(1);
rowsToKeep = and(rowsToKeep, tImu <= tSpan(2));

tRobot = tRobot(rowsToKeep);
tImu = tImu(rowsToKeep);
q = q(rowsToKeep,:);
z = z(rowsToKeep,:);

% Detect where time changes badly
hiDtRows = find(diff(tImu) > 0.05);
rowsToRemove = hiDtRows + (-10:10);
rowsToRemove = rowsToRemove(:);

rowsToKeep = true(size(tImu));
rowsToKeep(rowsToRemove) = false;

tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

% Now remove repeated values from the measurements
rowsToRemove = diff(tImu) == 0;
rowsToRemove = or(rowsToRemove, all(diff(z, 1, 1) == 0, 2));

rowsToKeep = true(size(tImu));
rowsToKeep(rowsToRemove) = false;

tImu = tImu(rowsToKeep);
z = z(rowsToKeep,:);

% Convert angular velocity to rad
z(:,4:6) = deg2rad(z(:,4:6));

% Estimate the sampling rate
sampleRate = length(tImu)/(tImu(end) - tImu(1));
fprintf('\nAverage sample rate: %.0f\n', sampleRate);

% Estimate qCov and zCov
numInteriorKnots = floor(length(tImu)./30);
zf = GetVectorSplineFunctions(z, tImu, 5, numInteriorKnots);
zFiltered = zf(tImu);
qf = GetVectorSplineFunctions(q, tRobot, 5, numInteriorKnots);
qFiltered = qf(tRobot);

zError = z - zFiltered;
qError = q - qFiltered;

zCov = cov(zError);
qCov = cov(qError);

fprintf('zCov: \n');
disp(zCov);
fprintf('qCov: \n');
disp(qCov);

PlotImuMeasurements(tRobot, q, qFiltered, tImu, z, zFiltered);
save(dataFilenameProcessed, 'tRobot', 'tImu', 'q', 'z');
end