function ComputeImuCalibration()

robotName = 'Puma560';
dataFile = 'SimulatedMeasurements.mat';
useRecursive = true;

ChangeRobot(robotName);
dataObj = load(dataFile);

thetaNominal = dataObj.thetaNominal;
thetaTruth = dataObj.thetaTruth;
thetaCov = dataObj.thetaCov;
t = dataObj.t;
q = dataObj.q;
qCov = dataObj.qCov;
z = dataObj.z;
zCov = dataObj.zCov;

if useRecursive
    [thetaStar, thetaStarCov] = CalibrateRecursive(t, q, z, zCov, thetaNominal, thetaCov, thetaTruth);
else
    [thetaStar, thetaStarCov] = CalibrateLeastSquares(t, q, z, zCov, thetaNominal, thetaCov);
    PlotCalibrationResults(thetaTruth, thetaStar, thetaStarCov);
end


end