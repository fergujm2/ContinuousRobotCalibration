clear;

% Suppose we know that a signal is of the form:
% i.e. f(t) = x1*sin(x2*t + x3) + x4*exp(t.*x5)
% Suppose that we don't know omega or phi either
n = 10;

% Define the ground truth coefficients
aTruth = 3.*rand(1, n)' + 1;
omegTruth = 5.*rand(1, n)';
phiTruth = 2*pi.*rand(1, n)';

xTruth = [aTruth; omegTruth; phiTruth];

f = @(t, x) sin((t')*(x((n + 1):(2*n))') + x((2*n + 1):(3*n))')*x(1:n);

% Sample the function
N = 500;
t = linspace(0, 5, N);
sigv2 = 0.1;
R = diag(sigv2*ones(1, N));
z = f(t, xTruth) + mvnrnd(zeros(N, 1), R)';

% % Batch process the data
RInv = inv(R);
x0 = xTruth + 1e0.*randn(size(xTruth));

xHatB = x0;
PHatB = 1e-3.*eye(size(xHatB,1));
while true
    [xHatBNew, PHatB] = EkfUpdate(xHatB, PHatB, 0, z, 0, @(x, u, t) x, @(x, tt) f(t, x), zeros(size(xHatB,1)), R);
    
    ndx = norm(xHatBNew - xHatB, inf);
    if ndx < 1e-3
        break
    end
    
    xHatB = xHatBNew;
end

figure(1);
clf;
hold on;

tS = linspace(0, 5, 1000);
zS = f(tS, xTruth);

plot(tS, zS, '-r', 'LineWidth', 1);
plot(t, z, '.k', 'MarkerSize', 15);
plot(tS, f(tS, xHatB), '-b', 'LineWidth', 1);

legend('Truth', 'Data', 'Estimated');
title('Batch Estimation of Function Coefficients');

% Recursively process the data
xHat = x0;
PHat = 1e0.*eye(size(xHat,1));

ind = randperm(N);

figure(2);
clf;
hold on;

plot(tS, zS, '-r', 'LineWidth', 1);
h1 = plot(t(ind(1:1)), z(ind(1:1)), '.k', 'MarkerSize', 15);
h2 = plot(tS, f(tS, xHat), '-b', 'LineWidth', 1);

legend('Truth', 'Data', 'Estimated');
title('Recursive Estimation of Function Coefficients');

for iii = 1:N
    ti = t(ind(iii));
    Ri = R(ind(iii),ind(iii));
    zi = z(ind(iii));
    
    [xHat, PHat] = EkfUpdate(xHat, PHat, 0, zi, ti, @(x, u, t) x, @(x, t) f(t, x), zeros(size(xHat,1)), Ri);
    
    h1.XData = t(ind(1:iii));
    h1.YData = z(ind(1:iii));
    
    h2.XData = tS;
    h2.YData = f(tS, xHat);
    
    drawnow();

    fprintf('Max singular value of P: %.6f\n', max(svd(PHat)));
end
