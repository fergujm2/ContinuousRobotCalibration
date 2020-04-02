function [t, q, alph, omeg] = GenerateImuMeasurements(qWayPts, sampleRate, tSpan, eTruth, gTruth, qCov, alphCov, omegCov, showPlot)
numMeas = sampleRate*(tSpan(2) - tSpan(1));

numJoints = size(qWayPts,2);
[y, C] = FitVectorQuinticSpline(qWayPts, tSpan(1), tSpan(2));

t = linspace(tSpan(1) + 0.1, tSpan(2) - 0.1, numMeas);
q = @(t) EvalVectorQuinticSpline(y, C, t);

qTruth = q(t);

[alphTruth, omegTruth] = ComputeImuMeasurements(q, t, eTruth, gTruth);

% Add noise to measurements
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);
alph = alphTruth + mvnrnd(zeros(1,3), alphCov, numMeas);
omeg = omegTruth + mvnrnd(zeros(1,3), omegCov, numMeas);

if showPlot
    h = figure(1);
    h.Color = [1,1,1];
    h.Position = [20, 50, 1500, 700];
    
    
    for iii = 1:length(t)
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