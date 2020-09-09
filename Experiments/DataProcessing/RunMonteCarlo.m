clear;

dataDir = 'Simulation';
filenames = {'OptimalTrajectory_Full_10n', 'OptimalTrajectory_Offset_10n', 'OptimalTrajectory_RobotParams_10n'};
timeSpans = [10, 20, 30, 60, 90, 120];
numCalibrations = 50;

for dataFilename = filenames
    for tSpan = timeSpans
        MonteCarloCalibration(dataDir, dataFilename{1}, tSpan, numCalibrations);
    end
end
