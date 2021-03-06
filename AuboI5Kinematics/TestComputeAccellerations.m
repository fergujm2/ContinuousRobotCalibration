clear;

% Define trajectory and sample points
q = @(t) (t.^3).*ones(1, 6);
qDot = @(t) (3.*t.^2).*ones(1, 6);
qDDot = @(t) (3*2.*t).*ones(1, 6);

N = 100000;
t = linspace(0, 1, N)';

e = 0.01.*randn(1, 6*7);

% Compute accellerations
qs = q(t);
qsDot = qDot(t);
qsDDot = qDDot(t);

[p, R, J, frames] = ComputeForwardKinematics(qs, e, false);
[vAll, aAll, omegAll, alphAll] = ComputeAccellerations(qs, qsDot, qsDDot, e);

v = squeeze(vAll(:,end,:))';
a = squeeze(aAll(:,end,:))';
omeg = squeeze(omegAll(:,end,:))';
alph = squeeze(alphAll(:,end,:))';

% Compute with finite differences
del = mean(diff(t,1));

vFd = diff(p, 1, 1)./del;
aFd = diff(vFd, 1, 1)./del;

omegFd = zeros(N - 1, 3);

for iii = 1:(N - 1)
    % Determine omega in world frame from RDot and R
    Ri = R(:,:,iii);
    omegFd(iii,:) = J(4:6,:,iii)*(qsDot(iii,:)');
end

alphFd = diff(omegFd, 1, 1)./del;

% Compare FD with computed
vError = v(1:(N - 1),:) - vFd;
aError = a(1:(N - 2),:) - aFd;
omegError = omeg(1:(N - 1),:) - omegFd;
alphError = alph(1:(N - 2),:) - alphFd;

figure(1);

subplot(4,1,1);
plot(t(1:(N - 1)), sqrt(sum(vError.^2, 2)));
ylabel('vError');

subplot(4,1,2);
plot(t(1:(N - 2)), sqrt(sum(aError.^2, 2)));
ylabel('aError');

subplot(4,1,3);
plot(t(1:(N - 1)), sqrt(sum(omegError.^2, 2)));
ylabel('omegaError');

subplot(4,1,4);
plot(t(1:(N - 2)), sqrt(sum(alphError.^2, 2)));
ylabel('alphError');
