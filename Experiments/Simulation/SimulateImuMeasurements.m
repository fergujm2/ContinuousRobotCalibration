function [tRobot, q, tImu, z, thetaTruth] = SimulateImuMeasurements(trajectoryFilename)

trajectoryFullFilename = fullfile('..', '..', 'Algorithm', 'OptimalTrajectoryGeneration', 'Output', trajectoryFilename);
sampleRate = 120;

dataObj = load(trajectoryFullFilename);
y = dataObj.y;
C = dataObj.C;
d = dataObj.d;
tSpan = dataObj.tSpan;

% Now, we need the end points to be zero
numZeros = 3;
C(:,(end - numZeros + 1):end) = repmat(C(:,1), 1, numZeros);

[thetaNominal, thetaCov] = GetThetaNominal();
thetaTruth = mvnrnd(thetaNominal, thetaCov)';

numMeas = sampleRate*(tSpan(2) - tSpan(1));

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

qf = @(t) EvalVectorSpline(y, C, d, t);
qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

tRobot = linspace(tSpan(1), tSpan(2), numMeas);

stdImuTiming = 0.001;
tImu = tRobot + stdImuTiming.*randn(size(tRobot));

zTruth = ImuMeasurementEquation(thetaTruth, tImu, qf, qDot, qDDot);
qTruth = qf(tRobot);

[zCov, qCov] = GetCovariances();

z = zTruth + mvnrnd(zeros(1,6), diag(zCov), numMeas);
q = qTruth + mvnrnd(zeros(1,6), diag(qCov), numMeas);
end