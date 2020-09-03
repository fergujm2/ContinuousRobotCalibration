function SimulateImuMeasurements()

sampleRate = 100;
numReps = 650;
trajectoryFilename = fullfile('..', '..', 'Algorithm', 'OptimalTrajectoryGeneration', 'Output', 'OptimalTrajectory_Full.mat');

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

t = linspace(tSpan(1), tSpan(2), numMeas);

zTruth = ImuMeasurementEquation(thetaTruth, t, qf, qDot, qDDot);
qTruth = qf(t);

[zCov, qCov] = GetCovariances();

z = zTruth + mvnrnd(zeros(1,6), zCov, numMeas);
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

tRobot = t;
tImu = t;

filename = fullfile('DataProcessed', 'SimulatedMeasurements.mat');
save(filename, 'thetaNominal', 'thetaTruth', 'tRobot', 'tImu', 'q', 'qCov', 'z', 'zCov');

PlotImuMeasurements(tRobot, q, qTruth, tImu, z, zTruth);

end