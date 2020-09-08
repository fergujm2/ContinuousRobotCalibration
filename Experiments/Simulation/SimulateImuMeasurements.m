function SimulateImuMeasurements()

sampleRate = 120;
numReps = 750;
% filename = 'OptimalTrajectory_Full_10n';
% filename = 'OptimalTrajectory_Offset_10n';
filename = 'OptimalTrajectory_RobotParams_10n';

trajectoryFilename = fullfile('..', '..', 'Algorithm', 'OptimalTrajectoryGeneration', 'Output', filename);

dataObj = load(trajectoryFilename);
A = dataObj.A;
B = dataObj.B;
T = dataObj.T;

tSpan = [0, T*numReps];

thetaNominal = GetThetaNominal();
thetaTruth = GetThetaTruth();

numMeas = sampleRate*(tSpan(2) - tSpan(1));
numJoints = size(A, 2);

[Ad, Bd] = DerVectorFourier(A, B, T);
[Add, Bdd] = DerVectorFourier(Ad, Bd, T);

qf = @(t) EvalVectorFourier(A, B, t, T);
qDot = @(t) EvalVectorFourier(Ad, Bd, t, T);
qDDot = @(t) EvalVectorFourier(Add, Bdd, t, T);

tRobot = linspace(tSpan(1), tSpan(2), numMeas);

stdImuTiming = 0.001;
tImu = tRobot + stdImuTiming.*randn(size(tRobot));

zTruth = ImuMeasurementEquation(thetaTruth, tImu, qf, qDot, qDDot);
qTruth = qf(tRobot);

[zCov, qCov] = GetCovariances();

z = zTruth + mvnrnd(zeros(1,6), zCov, numMeas);
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

filename = fullfile('DataProcessed', filename);
save(filename, 'thetaNominal', 'thetaTruth', 'tRobot', 'tImu', 'q', 'qCov', 'z', 'zCov');

PlotImuMeasurements(tRobot, q, qTruth, tImu, z, zTruth);

end