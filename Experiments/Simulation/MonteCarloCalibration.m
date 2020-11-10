function MonteCarloCalibration(trajectoryFilename, numCalibrations)

for iii = 1:numCalibrations
    [tRobot, q, tImu, z, thetaTruth] = SimulateImuMeasurements(trajectoryFilename);
    [thetaStar, CStar, y, d] = ComputeImuCalibration(tRobot, q, tImu, z);
    
    fprintf('\nCompleted %d calibrations.\n', iii);
    
    outputFilename = [trajectoryFilename, '_', datestr(now,'yymmdd_HHMMSSFFF')];
    outputFullFilename = fullfile('OutputCalibrations', outputFilename);
    parSave(outputFullFilename, thetaStar, thetaTruth, y, d, CStar);
end

end

function parSave(filename, thetaStar, thetaTruth, y, d, CStar)
    save(filename, 'thetaStar', 'thetaTruth', 'y', 'd', 'CStar');
end