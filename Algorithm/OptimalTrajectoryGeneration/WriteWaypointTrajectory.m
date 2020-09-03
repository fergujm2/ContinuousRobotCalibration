function WriteWaypointTrajectory(numWaypoints, simulateTrajectory)

% First choose a bunch of joint value waypoints
jointLimits = [-20, 20
               -20, 20
                60, 90
                80, 100
                80, 100
               -60, 60].*pi/180; % rad

jointLengths = jointLimits(:,2) - jointLimits(:,1);
jointMeans = mean(jointLimits, 2);
numJoints = size(jointLimits, 1);
qWaypoints = nan(numWaypoints, numJoints);

for iii = 1:numWaypoints
    qWaypoints(iii,:) = jointMeans + jointLengths.*(rand(numJoints, 1) - 0.5);
end

% Now that we have the waypoints, interpolate between them
qInterp = reshape(repmat(qWaypoints, 1, 2)', numJoints, [])';

stopTime = 2;
travelTime = 3;
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

csvFullFilename = fullfile('Output', 'WayPointTrajectory.csv');
writematrix([t', q], csvFullFilename);

if simulateTrajectory
    SimulateTrajectory(t, q, fullfile('Output', 'WayPointTrajectory.mp4'));
end

end