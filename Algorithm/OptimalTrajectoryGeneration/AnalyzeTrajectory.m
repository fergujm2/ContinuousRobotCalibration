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

tObs = 10:1:tSpan(end);
% tObs = [10, 15, 20];

for iii = 1:length(tObs)
    fprintf('Computing %.0f of %.0f observabilities...\n', iii, length(tObs));
    [~, thetaCov(:,:,iii)] = ComputeObservability(y, C, d, sampleRate, [3, tObs(iii)], [], inf);
end

fullFilenameOut = [fullFilename, '_Analyzed'];
save(fullFilenameOut, 'tObs', 'thetaCov');
end