function [t, q, alph, omeg, yb, db, Cba, Cbw] = GenerateImuMeasurements(qWayPts, sampleRate, tSpan, eTruth, gTruth, qCov, alphCov, omegCov, Qa, Qw, showPlot)
numMeas = sampleRate*(tSpan(2) - tSpan(1));
numJoints = size(qWayPts,2);

d = 5;

% qWayPts = zeros(size(qWayPts));
% qWayPts(:,1) = -linspace(-1,1,30);
% gTruth = [0;0;0];

[y, C] = FitVectorSpline(qWayPts, tSpan(1), tSpan(2), d);
[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

t = linspace(tSpan(1) + 0.1, tSpan(2) - 0.1, numMeas);

qTruth = EvalVectorSpline(y, C, d, t);
qDotTruth = EvalVectorSpline(yd, Cd, dd, t);
qDDotTruth = EvalVectorSpline(ydd, Cdd, ddd, t);

[alphTruth, omegTruth] = ComputeImuMeasurements(qTruth, qDotTruth, qDDotTruth, eTruth, gTruth);

baDot = mvnrnd(zeros(1,3), Qa, 4);
bwDot = mvnrnd(zeros(1,3), Qw, 4);

dbd = 3;
[ybDot, CbaDot] = FitVectorSpline(baDot, tSpan(1), tSpan(2), dbd);
[~, CbwDot] = FitVectorSpline(bwDot, tSpan(1), tSpan(2), dbd);

[yb, Cba, db] = IntVectorSpline(ybDot, CbaDot, dbd);
[~, Cbw, ~] = IntVectorSpline(ybDot, CbwDot, dbd);

alphBias = EvalVectorSpline(yb, Cba, db, t);
omegBias = EvalVectorSpline(yb, Cbw, db, t);

% Add noise and bias to measurements
q = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);
alph = alphTruth + alphBias + mvnrnd(zeros(1,3), alphCov, numMeas);
omeg = omegTruth + omegBias + mvnrnd(zeros(1,3), omegCov, numMeas);

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