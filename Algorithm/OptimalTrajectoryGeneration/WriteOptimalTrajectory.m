function WriteOptimalTrajectory(filename, simulateTrajectory)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);

A = dataObj.A;
B = dataObj.B;
T = dataObj.T;
T = T*1;

sampleRate = 300;
tSpan = [0, T];

numMeas = sampleRate*(tSpan(2) - tSpan(1));

t = linspace(tSpan(1), tSpan(2), numMeas);
q = EvalVectorFourier(A, B, t, T);

[~, trajectoryName, ~] = fileparts(fullFilename);
verboseFilename = [trajectoryName, '_', num2str(T), 'Sec_', num2str(sampleRate), 'Hz'];

csvFilename = [verboseFilename, '.csv'];
csvFullFilename = fullfile('Output', csvFilename);

writematrix([t', q], csvFullFilename);

videoFilename = [verboseFilename, '.mp4'];
videoFullFilename = fullfile('Output', videoFilename);

if simulateTrajectory
    SimulateTrajectory(t, q, videoFullFilename);
end

end