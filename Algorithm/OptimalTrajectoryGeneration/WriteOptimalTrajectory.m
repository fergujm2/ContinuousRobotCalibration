function WriteOptimalTrajectory(filename, simulateTrajectory)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);

y = dataObj.y;
C = dataObj.C;
d = dataObj.d;
tSpan = dataObj.tSpan;

% Now, we need the end points to be zero
C(:,(end - d + 1):end) = C(:,1:d);

sampleRate = 300;
numMeas = sampleRate*(tSpan(2) - tSpan(1));

t = linspace(tSpan(1), tSpan(2), numMeas);
q = EvalVectorSpline(y, C, d, t);

[~, trajectoryName, ~] = fileparts(fullFilename);
verboseFilename = [trajectoryName, '_', num2str(sampleRate), 'Hz'];

csvFilename = [verboseFilename, '.csv'];
csvFullFilename = fullfile('Output', csvFilename);

writematrix([t', q], csvFullFilename);

videoFilename = [verboseFilename, '.avi'];
videoFullFilename = fullfile('Output', videoFilename);

if simulateTrajectory
    SimulateTrajectory(t, q, videoFullFilename, 30);
end

end