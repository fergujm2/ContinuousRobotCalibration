function ReduceCollectedData()

dataDir = fullfile('..', '20200730_FirstDataSet');
dataFilename = 'ImuCalibrationDataOffset_10s_20200730_151522.mat';
% dataFilename = 'ImuCalibrationDataOffset_25s_20200730_152436.mat';
% dataFilename = 'ImuCalibrationDataFull_10s_20200730_145802.mat';
% dataFilename = 'ImuCalibrationDataFull_25s_20200730_153238.mat';
dataFilenameRaw = fullfile(dataDir, 'DataRaw', dataFilename);
dataFilenameProcessed = fullfile(dataDir, 'DataProcessed', dataFilename);

tSpan = [0, 270];

dataObj = load(dataFilenameRaw);

imuMsgData = dataObj.imuMsgData;
robotMsgData = dataObj.robotMsgData;

tRobot = robotMsgData(:,1);
qDes = robotMsgData(:,2:7);
q = robotMsgData(:,8:end);

tImu = imuMsgData(:,1);
w = imuMsgData(:,2:4);
a = imuMsgData(:,5:7);

% Shift timestamps from initial values
tRobot = tRobot - tRobot(1);
tImu = tImu - tImu(1);

% Determine rows to keep based on timespan of interest
rowsToKeep = tRobot > tSpan(1);
rowsToKeep = and(rowsToKeep, tRobot < tSpan(2));

tRobot = tRobot(rowsToKeep);
tImu = tImu(rowsToKeep);
q = q(rowsToKeep,:);
w = w(rowsToKeep,:);
a = a(rowsToKeep,:);

% Detect where time changes badly
hiDtRows = find(diff(tImu) > 0.05);
rowsToRemove = hiDtRows + (-10:10);
rowsToRemove = rowsToRemove(:);

rowsToKeep = true(size(tImu));
rowsToKeep(rowsToRemove) = false;

tImu = tImu(rowsToKeep);
w = w(rowsToKeep,:);
a = a(rowsToKeep,:);

% Now remove repeated values from the measurements
rowsToRemove = any(diff([tImu, w, a],1,1) == 0, 2);

rowsToKeep = true(size(tImu));
rowsToKeep(rowsToRemove) = false;

tImu = tImu(rowsToKeep);
w = w(rowsToKeep,:);
a = a(rowsToKeep,:);

% Convert angular velocity to rad
w = deg2rad(w);
z = [a, w];

% Estimate qCov and zCov
zf = GetVectorSplineFunctions(z, tImu, 5, floor(length(tImu)./30));
zFiltered = zf(tImu);
qf = GetVectorSplineFunctions(q, tRobot, 5, floor(length(tRobot)./30));
qFiltered = qf(tRobot);


PlotImuMeasurements(tRobot, q, qFiltered, tImu, z, zFiltered);

save(dataFilenameProcessed, 'tRobot', 'tImu', 'q', 'z');
end