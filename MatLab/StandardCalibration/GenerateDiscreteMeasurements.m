function [p, q] = GenerateDiscreteMeasurements(qTruth, eTruth, pCov, qCov, showPlot)
numMeas = size(qTruth,1);
numJoints = size(qTruth,2);

pTruth = ComputeForwardKinematics(qTruth, eTruth, false);

% Add noise to measurements
p = pTruth + mvnrnd(zeros(1,3), pCov, numMeas);
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

if showPlot
    figure(1);
    clf;
    hold on;
    ComputeForwardKinematics(zeros(1,6), eTruth, true);
    scatter3(pTruth(:,1), pTruth(:,2), pTruth(:,3), 20, 'Filled', 'MarkerFaceColor', 'red');
    scatter3(p(:,1), p(:,2), p(:,3), 20, 'Filled', 'MarkerFaceColor', 'blue');
    daspect([1,1,1]);
    grid on;
end

end