clear;

testDir = '20201016_FullTest';
filename = 'Calibration';

dataDir = fullfile('..', testDir, 'DataProcessed');
dataFullfilename = fullfile(dataDir, filename);
serialDataObj = load(dataFullfilename);

q = serialDataObj.q; % Robot joint values
tRobot = serialDataObj.tRobot;
z = serialDataObj.z;
tImu = serialDataObj.tImu;

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

