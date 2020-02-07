function [p, q] = GenerateDiscreteMeasurements(robot, numMeasurements, varargin)
numJoints = length(robot.randomConfiguration());

defaultPCov = zeros(3);
defaultQCov = zeros(numJoints);
defaultShowPlot = false;

ip = inputParser();

addOptional(ip, 'pCov', defaultPCov);
addOptional(ip, 'qCov', defaultQCov);
addOptional(ip, 'showPlot', defaultShowPlot);

parse(ip, varargin{:});

pCov = ip.Results.pCov;
qCov = ip.Results.qCov;
showPlot = ip.Results.showPlot;

tipBodyName = robot.BodyNames{end};

pTruth = zeros(numMeasurements, 3);
qTruth = zeros(numMeasurements, numJoints);

for iii = 1:numMeasurements
    qConfig = robot.randomConfiguration();
    tipTransform = robot.getTransform(qConfig, tipBodyName);
    
    for jjj = 1:numJoints
        qTruth(iii,jjj) = qConfig(jjj).JointPosition;
    end
    
    pTruth(iii,:) = tipTransform(1:3,4)';
    
    if showPlot
        figure(1);
        clf;
        robot.show(qConfig);
        hold on;
        scatter3(pTruth(1:iii,1), pTruth(1:iii,2), pTruth(1:iii,3), 20, 'Filled', 'MarkerFaceColor', 'red');
    end
end

% Add noise to measurements
p = pTruth + mvnrnd(zeros(1,3), pCov, numMeasurements);
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeasurements);

if showPlot
    figure(1);
    clf;
    robot.show(robot.homeConfiguration);
    hold on;
    scatter3(pTruth(:,1), pTruth(:,2), pTruth(:,3), 20, 'Filled', 'MarkerFaceColor', 'red');
    scatter3(p(:,1), p(:,2), p(:,3), 20, 'Filled', 'MarkerFaceColor', 'blue');
end

end