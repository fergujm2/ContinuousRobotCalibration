function SimulateImuMeasurements()
trajectoryFilename = fullfile('..', '..', 'Algorithm', 'OptimalTrajectoryGeneration', 'Output', 'OptimalTrajectory_Offset.mat');
% trajectoryFilename = fullfile('..', '20200730_FirstDataSet', 'Trajectories', 'OptimalTrajectoryFull.mat');

% rng(3);

[thetaNominal, thetaCov] = GetNominalTheta();
thetaTruth = mvnrnd(thetaNominal, thetaCov)';

T = 30;
sampleRate = 100;
tSpan = [0, T*105];

dataObj = load(trajectoryFilename);
A = dataObj.A;
B = dataObj.B;

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
qDotTruth = qDot(t);
qDDotTruth = qDDot(t);

% Add noise and scale to measurements
[zCov, qCov, qDotCov, qDDotCov] = GetCovariances();
[~, measCov3] = ComputeMeasurementCovariance(qTruth, qDotTruth, qDDotTruth, thetaTruth, zCov, qCov, qDotCov, qDDotCov);

z = zeros(size(zTruth));
parfor iii = 1:numMeas
    z(iii,:) = zTruth(iii,:) + mvnrnd(zeros(1,6), measCov3(:,:,iii), 1);
end

q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

tRobot = t;
tImu = t;

filename = fullfile('DataProcessed', 'SimulatedMeasurements.mat');
save(filename, 'thetaNominal', 'thetaTruth', 'thetaCov', 'tRobot', 'tImu', 'q', 'qCov', 'z', 'zCov');

% Write qDes for one period to path plan CSV file
sampleRateDes = 300;
tDes = linspace(tSpan(1), tSpan(1) + T, T*sampleRateDes);
qDes = qf(tDes);

[~, trajectoryName, ~] = fileparts(trajectoryFilename);
csvFilename = [trajectoryName, '_', num2str(T), 'Sec_', num2str(sampleRateDes), 'Hz', '.csv'];
writematrix([tDes', qDes], csvFilename);

PlotImuMeasurements(tRobot, q, qTruth, tImu, z, zTruth);

% if showPlot
%     alph = z(:,1:3);
%     omeg = z(:,4:6);
%     
%     [calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
% 
%     % Robot parameters
%     x = UnpackTheta(thetaTruth);
%     e = zeros(1,numParamsTotal);
%     e(calibBools) = e(calibBools) + x';
% 
%     h = figure(1);
%     h.Color = [1,1,1];
%     h.Position = [20, 50, 1500, 700];
%     
%     for iii = 1:10:length(t)
%         clf;
%         hold on;
%         grid on;
%         axis image;
%         xlim([-.5, .5]);
%         ylim([-.5, .5]);
%         zlim([-.2, 1.1]);
%         view([60, 30]);
%         
%         [p, R] = ComputeForwardKinematics(q(iii,:), e, true);
%         alphW = R*(alph(iii,:)');
%         omegW = R*(omeg(iii,:)');
%         alphW = alphW./norm(alphW)./5;
%         omegW = omegW./norm(omegW)./5;
%         quiver3(p(1), p(2), p(3), alphW(1), alphW(2), alphW(3), 'Linewidth', 1, 'Color', 'r');
%         quiver3(p(1), p(2), p(3), omegW(1), omegW(2), omegW(3), 'Linewidth', 1, 'Color', 'g');
%         
%         drawnow();
%     end
% end
end