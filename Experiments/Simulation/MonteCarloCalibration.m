function MonteCarloCalibration(filename, numCalibrations)

thetaNominal = GetThetaNominal();
thetaTruth = GetThetaTruth();

thetaStar = nan(length(thetaNominal), numCalibrations);
thetaStarCov = nan(length(thetaNominal),length(thetaNominal),numCalibrations);

for iii = 1:numCalibrations
    [tRobot, q, tImu, z] = SimulateImuMeasurements(filename);
    [thetaStar(:,iii), thetaStarCov(:,:,iii)] = ComputeImuCalibration(tRobot, q, tImu, z, thetaNominal);
    
    fprintf('\nCompleted %d of %d calibrations.\n', iii, numCalibrations);
end

outputFilename = [filename, '_', num2str(numCalibrations), 'Calib'];
outputFullFilename = fullfile('OutputCalibrations', outputFilename);
save(outputFullFilename, 'thetaStar', 'thetaStarCov', 'thetaNominal', 'thetaTruth');

end