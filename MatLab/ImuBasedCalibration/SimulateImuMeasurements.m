function SimulateImuMeasurements()
robotName = 'Puma560';
showPlot = false;

ChangeRobot(robotName);

[calibBools, numParams, numParamsTotal] = GetCalibInfo();

[thetaNominal, thetaCov] = GetNominalTheta();
thetaTruth = mvnrnd(thetaNominal, thetaCov)';

[xTruth, gTruth, tauTruth, sTruth] = UnpackTheta(thetaTruth);

eNominal = zeros(1,numParamsTotal);
eTruth = eNominal;
eTruth(calibBools) = eTruth(calibBools) + xTruth';

sampleRate = 21;
tSpan = [0, 220];

qCov = (1e-1*pi/180)^2*eye(6);

alphCov = (0.01)^2*eye(3);
omegCov = (0.01)^2*eye(3);
zCov = blkdiag(alphCov, omegCov);

% numWayPts = 50;
% qWayPts = SampleJointSpace(numWayPts);
dataObj = load('OptimalTrajectory.mat');
qWayPts = repmat(dataObj.qWayPts, 20, 1);

numMeas = sampleRate*(tSpan(2) - tSpan(1));
numJoints = size(qWayPts, 2);

d = 5;
[y, C] = FitVectorSpline(qWayPts, tSpan(1), tSpan(2), d);
[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

t = linspace(tSpan(1) + 20, tSpan(2) - 20, numMeas);
tOffset = t + tauTruth;

qTruth = EvalVectorSpline(y, C, d, t);

qOffset = EvalVectorSpline(y, C, d, tOffset);
qDotOffset = EvalVectorSpline(yd, Cd, dd, tOffset);
qDDotOffset = EvalVectorSpline(ydd, Cdd, ddd, tOffset);

[alphOffset, omegOffset] = ComputeImuMeasurements(qOffset, qDotOffset, qDDotOffset, eTruth, gTruth);

% Add noise and scale to measurements
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);
alph = sTruth(1).*alphOffset + mvnrnd(zeros(1,3), alphCov, numMeas);
omeg = sTruth(2).*omegOffset + mvnrnd(zeros(1,3), omegCov, numMeas);

z = [alph, omeg];

save('SimulatedMeasurements.mat', 'thetaNominal', 'thetaTruth', 'thetaCov', 't', 'q', 'qCov', 'z', 'zCov');

if showPlot
    h = figure(1);
    h.Color = [1,1,1];
    h.Position = [20, 50, 1500, 700];
    
    for iii = 1:5:length(t)
        clf;
        hold on;
        
        [p, R] = ComputeForwardKinematics(q(iii,:), eTruth, true);
        alphW = (R')*(alph(iii,:)');
        omegW = (R')*(omeg(iii,:)');
        alphW = alphW./norm(alphW)./5;
        omegW = omegW./norm(omegW)./5;
        quiver3(p(1), p(2), p(3), alphW(1), alphW(2), alphW(3), 'Linewidth', 1, 'Color', 'r');
        quiver3(p(1), p(2), p(3), omegW(1), omegW(2), omegW(3), 'Linewidth', 1, 'Color', 'g');
        
        grid on;
        axis image;
        xlim([-.3, 1]);
        ylim([-.8, .8]);
        zlim([-.2, 1.3]);
        view([130, 30]);
        drawnow();
    end
end
end