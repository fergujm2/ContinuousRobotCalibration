function MonteCarloCalibration(trajectoryFilename, numCalibrations)

thetaNominal = GetThetaNominal();

thetaTruth = nan(length(thetaNominal), numCalibrations); 
thetaStar = nan(length(thetaNominal), numCalibrations);
thetaStarCov = nan(length(thetaNominal),length(thetaNominal),numCalibrations);

for iii = 1:numCalibrations
    [tRobot, q, tImu, z, thetaTruth(:,iii)] = SimulateImuMeasurements(trajectoryFilename);
    
    thetaStar(:,iii) = ComputeImuCalibration(tRobot, q, tImu, z);
    
    fprintf('\nCompleted %d of %d calibrations.\n', iii, numCalibrations);
end

outputFilename = [trajectoryFilename, '_', num2str(numCalibrations), 'Calib'];
outputFullFilename = fullfile('OutputCalibrations', outputFilename);
save(outputFullFilename, 'thetaStar', 'thetaStarCov', 'thetaNominal', 'thetaTruth');

end