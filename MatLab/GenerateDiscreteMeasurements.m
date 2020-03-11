function [p, q] = GenerateDiscreteMeasurements(numMeas, jointLimits, xTruth, pCov, qCov, showPlot)
numJoints = 6;
qTruth = zeros(numMeas,numJoints);
            
for iii = 1:numJoints
    qLo = jointLimits(iii,1);
    qHi = jointLimits(iii,2);
    
    qTruth(:,iii) = (qHi - qLo)*rand(1,numMeas) + qLo;
end

pTruth = zeros(numMeas,3);
for iii = 1:numMeas
    pTruth(iii,:) = ComputeForwardKinematics(qTruth(iii,:), xTruth, false);
end

% Add noise to measurements
p = pTruth + mvnrnd(zeros(1,3), pCov, numMeas);
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

if showPlot
    figure(1);
    clf;
    hold on;
    ComputeForwardKinematics(zeros(1,6), xTruth, true);
    scatter3(pTruth(:,1), pTruth(:,2), pTruth(:,3), 20, 'Filled', 'MarkerFaceColor', 'red');
    scatter3(p(:,1), p(:,2), p(:,3), 20, 'Filled', 'MarkerFaceColor', 'blue');
    daspect([1,1,1]);
    grid on;
end

end