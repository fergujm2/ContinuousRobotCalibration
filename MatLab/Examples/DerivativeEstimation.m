clear;

sampleRate = 100;
tSpan = [0, 180];

numWayPts = 100;
numJoints = 1;

qWayPts = pi.*rand(numWayPts, numJoints);
numMeas = sampleRate*(tSpan(2) - tSpan(1));

qCov = (1e-1*pi/180)^2*eye(numJoints);

[q, qDot, qDDot] = FitJointValueFunctions(qWayPts, tSpan);

t = linspace(tSpan(1) + 5, tSpan(2) - 5, numMeas);

qTruth = q(t);
qDotTruth = qDot(t);
qDDotTruth = qDDot(t);

T = 1/sampleRate;

% Define state equations
% F = [1, T, 1/2*T^2, 1/6*T^3
%      0, 1, T, 1/2*T^2 
%      0, 0, 1, T
%      0, 0, 0, 1];

sigW = 5;
n = 5;

Qij = @(ii, jj) sigW^2/factorial(n - ii)/factorial(n - jj)/(2*n - ii - jj + 1)*T^(2*n - ii - jj + 1);
F1j = @(jj) 1/factorial(jj)*T^(jj);

F = zeros(n);
H = [1, zeros(1, n - 1)];

for jjj = 1:n
    F(1,jjj) = F1j(jjj-1);
end

for iii = 2:n
    F(iii,iii:end) = F(1,1:(end - iii + 1));
end

Q = zeros(n);

for iii = 1:n
    for jjj = 1:n
        Q(iii,jjj) = Qij(iii,jjj);
    end
end

% G = [1/6*T^3; 1/2*T^2; T];
% Q0 = G*(G')*sigW^2;
R = qCov;

% Measurements, position only
y = qTruth + mvnrnd(zeros(1,numJoints), qCov, numMeas);

% Initialize estimates
xHat = zeros(n, numMeas);
pHat = eye(n);
pHatNorm = zeros(1, numMeas);

% Run filter
for k = 2:numMeas
    [xHat(:,k), pHat] = KalmanUpdate(xHat(:,k-1), pHat, 0, y(k-1,:)', F, 0, H, Q, R);
    pHatNorm(k) = norm(pHat, 2);
end

figure(1);
clf;
hold on;

plot(t, qTruth);
plot(t, y, '.');

legend('Truth', 'Measurements');

figure(2);
clf;

subplot(3,1,1);
title('Derivative Estimation Using Kalman Filter');
hold on;

plot(t, qTruth);
plot(t, xHat(1,:));

ylabel('x');
legend('Truth', 'Estimated');

subplot(3,1,2);
hold on;

plot(t, qDotTruth);
plot(t, xHat(2,:));

ylabel('xDot');
legend('Truth', 'Estimated');

subplot(3,1,3);
hold on;

plot(t, qDDotTruth);
plot(t, xHat(3,:));

ylabel('xDDot');
legend('Truth', 'Estimated');


d = 5;
[qf, qDot, qDDot] = FitJointValueFunctions(y, [t(1), t(end)], floor(length(t)./10), 1e-7);

xHatSpline = [qf(t)'
              qDot(t)'
              qDDot(t)'];

figure(3);
clf;

subplot(3,1,1);
hold on;
title('Derivative Estimation Using B-Splines');

plot(t, qTruth);
plot(t, xHatSpline(1,:));

ylabel('x');
legend('Truth', 'Estimated');

subplot(3,1,2);
hold on;

plot(t, qDotTruth);
plot(t, xHatSpline(2,:));

ylabel('xDot');
legend('Truth', 'Estimated');

subplot(3,1,3);
hold on;

plot(t, qDDotTruth);
plot(t, xHatSpline(3,:));

ylabel('xDDot');
legend('Truth', 'Estimated');

% It seems like when you can, it's better to fit to a continuous function
% and differentiate than using a recursive estimation technique for
% estimation of derivatives. For splines specifically, this may be due to
% their power in estimation of derivatives.
