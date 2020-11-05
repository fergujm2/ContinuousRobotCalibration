function AnalyzeTrajectory(filename)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);

y = dataObj.y;
C = dataObj.C;
d = dataObj.d;
tSpan = dataObj.tSpan;
sampleRate = dataObj.sampleRate;

% Now, we need the end points to be zero
numZeros = 3;
C(:,(end - numZeros + 1):end) = repmat(C(:,1), 1, numZeros);

tObs = 0:5:tSpan(end);
% tObs = [0, 5, 10, 15];

[~, thetaCovInit] = GetThetaNominal();
thetaCov(:,:,1) = thetaCovInit;

for iii = 2:length(tObs)
    fprintf('Computing %.0f of %.0f observabilities...\n', iii, length(tObs));
    [~, thetaCov(:,:,iii)] = ComputeObservability(y, C, d, sampleRate, [1, tObs(iii)], [], thetaCovInit);
end

fullFilenameOut = [fullFilename, '_Analyzed'];
save(fullFilenameOut, 'tObs', 'thetaCov');
end