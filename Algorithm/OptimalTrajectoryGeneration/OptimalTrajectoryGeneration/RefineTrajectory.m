function RefineTrajectory(filename)

fullFilename = fullfile('Output', filename);
dataObj = load(fullFilename);

y = dataObj.y;
C0 = dataObj.C;
d = dataObj.d;
sampleRate = dataObj.sampleRate;

tSpan = [3, 240];

obj = @(C) ComputeObservability(y, C, d, sampleRate, tSpan, tSpan, inf, 1e-3);
    
% Refine new set of points with pattern search
options = optimoptions('patternsearch');
options.Display = 'iter';
options.MaxIterations = 150;
options.InitialMeshSize = 0.5;
options.MaxMeshSize = 0.5;
options.UseParallel = true;
options.PollMethod = 'GPSPositiveBasisNp1';
options.UseCompletePoll = true;
options.Cache = 'on';
options.PlotFcn = {@psplotbestf, @psplotbestx, @psplotmeshsize};
    
C = patternsearch(obj, C0, [], [], [], [], [], [], [], options);

[~, thetaCov] = obj(C);

fullFilenameOut = [fullFilename, '_Refined'];
tSpan = dataObj.tSpan;

save(fullFilenameOut, 'y', 'C', 'd', 'sampleRate', 'tSpan', 'thetaCov');

% [~, thetaCov] = ComputeObservability(y, C, d, sampleRate, tSpanFull, tSpanJointLimits);



end