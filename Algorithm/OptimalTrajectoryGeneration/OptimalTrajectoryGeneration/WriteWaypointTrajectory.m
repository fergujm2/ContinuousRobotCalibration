function WriteWaypointTrajectory(numWaypoints, stopTime, travelTime, simulateTrajectory)

% First choose a bunch of joint value waypoints
jointLimits = GetJointLimits();
jointLengths = deg2rad([20, 20, 20, 120, 40, 120]');

jointMeans = mean(jointLimits, 2);
numJoints = size(jointLimits, 1);
qWaypoints = nan(numWaypoints, numJoints);

for iii = 1:numWaypoints
    qWaypoints(iii,:) = jointMeans + jointLengths.*(2.*rand(numJoints, 1) - 1);
end

% Now that we have the waypoints, interpolate between them
qInterp = reshape(repmat(qWaypoints, 1, 2)', numJoints, [])';

T = travelTime + stopTime;

tSpan = [0, T*(numWaypoints - 1)];
tCycles = tSpan(1):T:tSpan(2);
tInterp = reshape([tCycles; tCycles + stopTime], 1, []);

sampleRate = 300;
numSamples = (tSpan(2) - tSpan(1))*sampleRate;

t = linspace(tSpan(1), tSpan(2), numSamples);
q = interp1(tInterp, qInterp, t);

windowSize = 300;
B = 1/windowSize*ones(windowSize, 1);
q = filter(B, 1, q);

q(1:windowSize, :) = ones(windowSize,1)*qWaypoints(1,:);

plot(t, q);

filename = sprintf('WayPointTrajectory_N%.0f_Stop%.0f_Travel%.0f', numWaypoints, stopTime, travelTime);
fullFilename = fullfile('Output', filename);

writematrix([t', q], fullFilename);

if simulateTrajectory
    SimulateTrajectory(t, q, fullFilename, 1);
end

end