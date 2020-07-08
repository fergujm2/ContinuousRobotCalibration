function SimulateImuMeasurements()
robotName = 'HebiX';
showPlot = false;

% rng(3);

ChangeRobot(robotName);

[thetaNominal, thetaCov] = GetNominalTheta();
thetaTruth = mvnrnd(thetaNominal, thetaCov)';

T = 5;
sampleRate = 100;
tSpan = [0, 50*T];

dataObj = load('OptimalTrajectory.mat');
A = dataObj.A;
B = dataObj.B;

numMeas = sampleRate*(tSpan(2) - tSpan(1));
numJoints = size(A, 2);

qCov = (1e-1*pi/180)^2*eye(numJoints);

alphCov = (0.01)^2*eye(3);
omegCov = (0.01)^2*eye(3);
zCov = blkdiag(alphCov, omegCov);

% [qf, qDot, qDDot] = FitJointValueFunctions(qWayPts, tSpan);
    
[Ad, Bd] = DerVectorFourier(A, B, T);
[Add, Bdd] = DerVectorFourier(Ad, Bd, T);

qf = @(t) EvalVectorFourier(A, B, t, T);
qDot = @(t) EvalVectorFourier(Ad, Bd, t, T);
qDDot = @(t) EvalVectorFourier(Add, Bdd, t, T);

t = linspace(tSpan(1), tSpan(2), numMeas);

[~, alph, omeg] = ImuMeasurementEquation(thetaTruth, t, qf, qDot, qDDot);
qTruth = qf(t);
zTruth = [alph, omeg];

% Add noise and scale to measurements
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);
z = zTruth + mvnrnd(zeros(1,6), zCov, numMeas);

save('SimulatedMeasurements.mat', 'thetaNominal', 'thetaTruth', 'thetaCov', 't', 'q', 'qCov', 'z', 'zCov');

if showPlot
    alph = z(:,1:3);
    omeg = z(:,4:6);
    
    [calibBools, ~, numParamsTotal] = GetRobotCalibInfo();

    % Robot parameters
    x = UnpackTheta(thetaTruth);
    e = zeros(1,numParamsTotal);
    e(calibBools) = e(calibBools) + x';

    h = figure(1);
    h.Color = [1,1,1];
    h.Position = [20, 50, 1500, 700];
    
    for iii = 1:5:length(t)
        clf;
        hold on;
        
        [p, R] = ComputeForwardKinematics(q(iii,:), e, true);
        alphW = (R')*(alph(iii,:)');
        omegW = (R')*(omeg(iii,:)');
        alphW = alphW./norm(alphW)./5;
        omegW = omegW./norm(omegW)./5;
        quiver3(p(1), p(2), p(3), alphW(1), alphW(2), alphW(3), 'Linewidth', 1, 'Color', 'r');
        quiver3(p(1), p(2), p(3), omegW(1), omegW(2), omegW(3), 'Linewidth', 1, 'Color', 'g');
        
        grid on;
        axis image;
        xlim([-4, 4]);
        ylim([-1, 4]);
        zlim([-1, 4]);
        view([130, 30]);
        drawnow();
    end
end
end