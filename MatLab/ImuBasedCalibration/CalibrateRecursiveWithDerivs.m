function [thetaStar, thetaStarCov] = CalibrateRecursiveDerivEst(t, q, qCov, z, zCov, thetaNominal, thetaCov)

% Our state vector for the estimation problem is 
%   xHat = [q1, ..., q6, qDot1, ..., qDot6, qDDot1, ..., qDDot6, ...
%           x1, ..., xn, g1, g2, g3, tau, s1, s2]';

T = mean(diff(t));
numJoints = size(q, 2);
N = length(t);

lengthTheta = length(thetaNominal);
n = 3*numJoints + lengthTheta;

G = ([1/6*T^3; 1/2*T^2; T]*ones(1, numJoints))';
G = G(:);

sigW = 10;
Vq = G*(G')*sigW^2;
V = blkdiag(Vq, 0.0001^2.*eye(lengthTheta));
W = blkdiag(qCov, zCov);

% Initialize estimates
xHat = [q(1,:)'; zeros(2*numJoints, 1); thetaNominal];
pHat = blkdiag(1000.*eye(3*numJoints), (0.001^2).*thetaCov);
pHatNorm = zeros(1, N);

for iii = 2:N
    ti = t(iii);
    zi = [q(iii,:), z(iii,:)]';
    
    [xHat, pHat] = EkfUpdate(xHat, pHat, 0, zi, ti, ...
                             @(x, u, t) stateEquation(x, T), ...
                             @(x, t) measurementEquation(x), V, W);
    
    msv = max(svd(pHat));
    fprintf('Max singular value of P: %.6f\n', msv);
    if msv < 1e-4
        break
    end 
end

thetaStar = xHat((end - lengthTheta + 1):end);
thetaStarCov = pHat((end - lengthTheta + 1):end, (end - lengthTheta + 1):end);

end

function xHatP = stateEquation(xHat, T)
    qOld = xHat(1:6);
    qDotOld = xHat(7:12);
    qDDotOld = xHat(13:18);
    
    qNew = qOld + T.*qDotOld + 1/2*(T^2).*qDDotOld;
    qDotNew = qDotOld + T.*qDDotOld;
    qDDotNew = qDDotOld;
    
    xHatP = zeros(size(xHat));
    xHatP(1:18) = [qNew; qDotNew; qDDotNew];
    xHatP(19:end) = xHat(19:end);
end

function y = measurementEquation(xHat)
    q = xHat(1:6);
    qDot = xHat(7:12);
    qDDot = xHat(13:18);
    theta = xHat(19:end);
    
    [calibBools, numParams, numParamsTotal] = GetCalibInfo();

    % Robot parameters
    x = theta(1:numParams);
    e = zeros(1,numParamsTotal);
    e(calibBools) = e(calibBools) + x';

    % Gravity direction
    gw = theta((numParams + 1):(numParams + 3));

    % System time offset
    tau = theta(numParams + 4);
    
    % Apply 2nd order approximation to joint value functions
    qData = q + tau.*qDot + 1/2*tau^2.*qDDot;
    qDotData = qDot + tau.*qDDot;
    qDDotData = qDDot;

    [alph, omeg] = ComputeImuMeasurements(qData', qDotData', qDDotData', e, gw);

    % Data scale factors
    s = theta((numParams + 5):end);
    alph = s(1).*alph;
    omeg = s(2).*omeg;
    
    y = [q; alph'; omeg'];
end