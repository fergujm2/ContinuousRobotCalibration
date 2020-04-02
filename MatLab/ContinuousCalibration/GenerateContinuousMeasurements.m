function [p, q, t] = GenerateContinuousMeasurements(qWayPts, sampleRate, tSpan, dTruth, eTruth, pCov, qCov, showPlot)
numMeas = sampleRate*(tSpan(2) - tSpan(1));

numJoints = size(qWayPts,2);
[y, C] = FitVectorQuinticSpline(qWayPts, tSpan(1), tSpan(2));

t = linspace(tSpan(1) + abs(dTruth), tSpan(2) - abs(dTruth), numMeas);
qTruth = EvalVectorQuinticSpline(y, C, t);

tOffset = t + dTruth;
qOffset = EvalVectorQuinticSpline(y, C, tOffset);

pTruth = ComputeForwardKinematics(qTruth, eTruth, false);
pOffset = ComputeForwardKinematics(qOffset, eTruth, false);

% Add noise to measurements
p = pOffset + mvnrnd(zeros(1,3), pCov, numMeas);
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

if showPlot
    figure(1);
    clf;
    hold on;
    
    ComputeForwardKinematics(zeros(1,6), eTruth, true);
    scatter3(pTruth(:,1), pTruth(:,2), pTruth(:,3), 30, 'MarkerEdgeColor', 'red');
    scatter3(p(:,1), p(:,2), p(:,3), 20, 'Filled', 'MarkerFaceColor', 'black');
    grid on;
    axis image;
    view([130, 30]);
end

end