clear;

xP = [0.1, -1.5, 5.9, -4.5];
xDotP = polyder(xP);
xDDotP = polyder(xDotP);

T = 1/30;
t = 0:T:10;
N = length(t);

xTruth = polyval(xP, t) + sin(2.*t);
xDotTruth = polyval(xDotP, t) + 2.*cos(2.*t);
xDDotTruth = polyval(xDDotP, t) - 4.*sin(2.*t);

% Define state equations
F = [1, T, 1/2*T^2; 0, 1, T; 0, 0, 1];
G = [1/6*T^3; 1/2*T^2; T];
H = [1, 0, 0];

sigW = 10;
sigV = 0.05;

Q = G*(G')*sigW^2;
R = sigV^2;

% Measurements, position only
z = xTruth + mvnrnd(zeros(1,1), R, N)';

% Initialize estimates
xHat = zeros(3, N);
pHat = eye(3);
pHatNorm = zeros(1, N);

% Sample noise
w = mvnrnd(zeros(3,1), Q, N)';
v = mvnrnd(zeros(1,1), R, N)';

% Run filter
for k = 2:N
    xHatP = F*xHat(:,k - 1);
    pHatP = F*pHat*(F') + Q;
    
    y = z(k) - H*xHatP;
    S = H*pHatP*(H') + R;
    K = pHatP*(H')*inv(S);
    
    xHat(:,k) = xHatP + K*y;
    pHat = (eye(3) - K*H)*pHatP;
    pHatNorm(k) = norm(pHat, 2);
end

figure(1);
clf;
hold on;

plot(t, xTruth);
plot(t, z, '.');

legend('Truth', 'Measurements');

figure(2);
clf;

subplot(3,1,1);
title('Derivative Estimation Using Kalman Filter');
hold on;

plot(t, xTruth);
plot(t, xHat(1,:));

ylabel('x');
legend('Truth', 'Estimated');

subplot(3,1,2);
hold on;

plot(t, xDotTruth);
plot(t, xHat(2,:));

ylabel('xDot');
legend('Truth', 'Estimated');

subplot(3,1,3);
hold on;

plot(t, xDDotTruth);
plot(t, xHat(3,:));

ylabel('xDDot');
legend('Truth', 'Estimated');


d = 5;
[yq, Cq] = LsqFitVectorSpline(z', t(1), t(end), d, 20);
[yqd, Cqd, dd] = DerVectorSpline(yq, Cq, d);
[yqdd, Cqdd, ddd] = DerVectorSpline(yqd, Cqd, dd);

xHatSpline = [EvalVectorSpline(yq, Cq, d, t)';
              EvalVectorSpline(yqd, Cqd, dd, t)';
              EvalVectorSpline(yqdd, Cqdd, ddd, t)'];

figure(3);
clf;

subplot(3,1,1);
hold on;
title('Derivative Estimation Using B-Splines');

plot(t, xTruth);
plot(t, xHatSpline(1,:));

ylabel('x');
legend('Truth', 'Estimated');

subplot(3,1,2);
hold on;

plot(t, xDotTruth);
plot(t, xHatSpline(2,:));

ylabel('xDot');
legend('Truth', 'Estimated');

subplot(3,1,3);
hold on;

plot(t, xDDotTruth);
plot(t, xHatSpline(3,:));

ylabel('xDDot');
legend('Truth', 'Estimated');

% It seems like when you can, it's better to fit to a continuous function
% and differentiate than using a recursive estimation technique for
% estimation of derivatives. For splines specifically, this may be due to
% their power in estimation of derivatives.
