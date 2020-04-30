clear;

% Suppose we know that a signal is a linear combination of sinusoids
% i.e. f(t) = a1*sin(omeg1*t + phi1) + ...
% Suppose that we know omeg and phi, but we do not know the coefficients
n = 10;
omeg = linspace(1, 5, n);
phi = linspace(0, 2*pi, n);

% Define the ground truth coefficients
aTruth = randn(1, n)';

h = @(t) sin((t')*omeg + phi);
f = @(t, a) h(t)*a;

% Sample the function
N = 50;
t = linspace(0, 5, N);
sigv2 = 0.01;
R = diag(sigv2*ones(1, N));
z = f(t, aTruth) + mvnrnd(zeros(N, 1), R)';

% Batch process the data
H = h(t);
RInv = inv(R);
PHat = inv((H')*RInv*H);
aHatB = PHat*(H')*RInv*z;

figure(1);
clf;
hold on;

tS = linspace(0, 5, 1000);
zS = f(tS, aTruth);

plot(tS, zS, '-r', 'LineWidth', 1);
plot(t, z, '.k', 'MarkerSize', 15);
plot(tS, f(tS, aHatB), '-b', 'LineWidth', 1);

legend('Truth', 'Data', 'Estimated');
title('Batch Estimation of Function Coefficients');

% Recursively process the data
pHat = diag(1e3.*ones(1, n));
aHat = zeros(n, 1);

ind = randperm(N);

for iii = 1:N
    Hi = H(ind(iii),:);
    Ri = R(ind(iii),ind(iii));
    zi = z(ind(iii));
    
    pHatNew = inv(inv(pHat) + (Hi')*inv(Ri)*Hi);
    aHat = pHatNew*inv(pHat)*aHat + pHatNew*(Hi')*inv(Ri)*zi;
    pHat = pHatNew;
    
    figure(2);
    clf;
    hold on;
    
    plot(tS, zS, '-r', 'LineWidth', 1);
    plot(t(ind(1:iii)), z(ind(1:iii)), '.k', 'MarkerSize', 15);
    plot(tS, f(tS, aHat), '-b', 'LineWidth', 1);
    
    legend('Truth', 'Data', 'Estimated');
    title('Recursive Estimation of Function Coefficients');
    
    fprintf('Max singular value of P: %.6f\n', max(svd(pHat)));
end
