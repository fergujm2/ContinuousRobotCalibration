function [p, q, t] = GenerateContinuousMeasurements(robot, numMeas, numWayPts, a, b, varargin)
numJoints = length(robot.randomConfiguration());

defaultPCov = zeros(3);
defaultQCov = zeros(numJoints);
defaultTau = 0;
defaultShowPlot = false;

ip = inputParser();

addOptional(ip, 'pCov', defaultPCov);
addOptional(ip, 'qCov', defaultQCov);
addOptional(ip, 'tau', defaultTau);
addOptional(ip, 'showPlot', defaultShowPlot);

parse(ip, varargin{:});

pCov = ip.Results.pCov;
qCov = ip.Results.qCov;
tau = ip.Results.tau;
showPlot = ip.Results.showPlot;

qWayPts = zeros(numWayPts, numJoints);

for iii = 1:numWayPts
    qConfig = robot.randomConfiguration();
    for jjj = 1:numJoints
        qWayPts(iii,jjj) = qConfig(jjj).JointPosition;
    end
end

[y, C] = FitVectorQuinticSpline(qWayPts, a, b);

t = linspace(a + abs(tau), b - abs(tau), numMeas);
qTruth = EvalVectorQuinticSpline(y, C, t);

tOffset = t + tau;
qOffset = EvalVectorQuinticSpline(y, C, tOffset);

pOffset = zeros(numMeas, 3);
pTruth = zeros(numMeas, 3);

tipBodyName = robot.BodyNames{end};

for iii = 1:numMeas
    qConfig = robot.randomConfiguration();

    for jjj = 1:numJoints
        qConfig(jjj).JointPosition = qOffset(iii,jjj);
    end
    
    tipTransformOffset = robot.getTransform(qConfig, tipBodyName);
    pOffset(iii,:) = tipTransformOffset(1:3,4)';
    
    for jjj = 1:numJoints
        qConfig(jjj).JointPosition = qTruth(iii,jjj);
    end
    
    tipTransformTruth = robot.getTransform(qConfig, tipBodyName);
    pTruth(iii,:) = tipTransformTruth(1:3,4)';
    
%     if showPlot
%         figure(1);
%         clf;
%         robot.show(qConfig);
%         hold on;
%         drawnow;
%         scatter3(pTruth(1:iii,1), pTruth(1:iii,2), pTruth(1:iii,3), 20, 'Filled', 'MarkerFaceColor', 'red');
%     end
end

% Add noise to measurements
p = pOffset + mvnrnd(zeros(1,3), pCov, numMeas);
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

if showPlot
    figure(1);
    clf;
    robot.show(robot.homeConfiguration());
    hold on;
    scatter3(pTruth(:,1), pTruth(:,2), pTruth(:,3), 30, 'MarkerEdgeColor', 'red');
    scatter3(p(:,1), p(:,2), p(:,3), 20, 'Filled', 'MarkerFaceColor', 'black');
end

end