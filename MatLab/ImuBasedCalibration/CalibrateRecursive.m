function [thetaStar, thetaStarCov] = CalibrateRecursive(t, q, z, zCov, thetaNominal, thetaCov, thetaTruth)

[qs, qDots, qDDots] = FitJointValueFunctions(q, [t(1), t(end)], floor(length(t)./10), 1e-6);

numJoints = size(q, 2);
numMeas = size(q, 1);

T = mean(diff(t));

sigW = 20;
n = 5;

Qij = @(ii, jj) sigW^2/factorial(n - ii)/factorial(n - jj)/(2*n - ii - jj + 1)*T^(2*n - ii - jj + 1);
F1j = @(jj) 1/factorial(jj)*T^(jj);

F0 = zeros(n);

for jjj = 1:n
    F0(1,jjj) = F1j(jjj-1);
end

for iii = 2:n
    F0(iii,iii:end) = F0(1,1:(end - iii + 1));
end

Q0 = zeros(n);

for iii = 1:n
    for jjj = 1:n
        Q0(iii,jjj) = Qij(iii,jjj);
    end
end

R0 = (1e-1*pi/180)^2;

H = zeros(numJoints, numJoints*n);

for iii = 1:numJoints
    a = n*(iii - 1) + 1;
    H(iii,a) = 1;
end

R = R0*eye(numJoints);

F = F0;
Q = Q0;

for iii = 1:(numJoints - 1)
    F = blkdiag(F, F0);
    Q = blkdiag(Q, Q0);
end

% Initialize estimates
xHat = zeros(n*numJoints, numMeas);
pHat = eye(n*numJoints);

% Run filter
for k = 2:numMeas
    [xHat(:,k), pHat] = KalmanUpdate(xHat(:,k-1), pHat, 0, q(k,:)', F, 0, H, Q, R);
end

qf = @(tt) interp1(t, xHat(1:n:end,:)', tt);
qDot = @(tt) interp1(t, xHat(2:n:end,:)', tt);
qDDot = @(tt) interp1(t, xHat(3:n:end,:)', tt);

% Trim the data on the front and back
numMeas = length(t);
numTrim = floor(numMeas*0.05);
t = t(numTrim:(end - numTrim));
q = q(numTrim:(end - numTrim),:);
z = z(numTrim:(end - numTrim),:);

plot(t, qf(t));
drawnow();

qError = qf(t) - q;
fprintf('\nmax(max(qError)): %f deg \n\n', max(max(qError))*180/pi); 

N = length(t);

lengthTheta = length(thetaNominal);

V = 0.000001^2.*eye(lengthTheta);
W = 50.0^2.*zCov;

% Initialize estimates
xHat = thetaNominal;
pHat = (10^2).*thetaCov;
pHatNorm = zeros(1, N);

for iii = 2:N
    ti = t(iii);
    zi = z(iii,:)';
    
    [xHat, pHat] = EkfUpdate(xHat, pHat, 0, zi, ti, ...
                             @(x, u, t) x, ...
                             @(x, t) ImuMeasurementEquation(x, t, qf, qDot, qDDot), V, W);
    
%     msv = max(svd(pHat));
%     fprintf('Max singular value of P: %.6f\n', msv);
%     if msv < 1e-5
%         break
%     end 
    
    if mod(iii, 50) == 0
        PlotCalibrationResults(thetaTruth, xHat, pHat);
    end
end

thetaStar = xHat;
thetaStarCov = pHat;

end