function MonteCarloCalibration(trajectoryFilename, numCalibrations)

thetaNominal = GetThetaNominal();


for iii = 1:numCalibrations
    [tRobot, q, tImu, z, thetaTruth] = SimulateImuMeasurements(trajectoryFilename);
    
    thetaStar = ComputeImuCalibration(tRobot, q, tImu, z);
    
    outputFilename = [trajectoryFilename, '_', datestr(now,'yymmdd_HHMMSSFFF')];
    outputFullFilename = fullfile('OutputCalibrations', outputFilename);
    save(outputFullFilename, 'thetaStar', 'thetaNominal', 'thetaTruth');
    
    fprintf('\nCompleted %d of %d calibrations.\n', iii, numCalibrations);
end



end