function [thetaStar, thetaStarCov] = CalibrateRecursive(t, q, z, zCov, thetaNominal, thetaCov, thetaTruth)

d = 5;
[yq, Cq] = LsqFitVectorSpline(q, t(1), t(end), d, floor(length(t)./10), 1e-6);
[yqd, Cqd, dd] = DerVectorSpline(yq, Cq, d);
[yqdd, Cqdd, ddd] = DerVectorSpline(yqd, Cqd, dd);

qf = @(t) EvalVectorSpline(yq, Cq, d, t);
qDot = @(t) EvalVectorSpline(yqd, Cqd, dd, t);
qDDot = @(t) EvalVectorSpline(yqdd, Cqdd, ddd, t);

% Trim the data on the front and back
numMeas = length(t);
numTrim = floor(numMeas*0.05);
t = t(numTrim:(end - numTrim));
q = q(numTrim:(end - numTrim),:);
z = z(numTrim:(end - numTrim),:);

qError = qf(t) - q;
fprintf('\nmax(max(qError)): %f deg \n\n', max(max(qError))*180/pi);

N = length(t);

lengthTheta = length(thetaNominal);

V = 0.00001^2.*eye(lengthTheta);
W = 1.0^2.*zCov;

% Initialize estimates
xHat = thetaNominal;
pHat = (1.0^2).*thetaCov;
pHatNorm = zeros(1, N);

for iii = 2:N
    ti = t(iii);
    zi = z(iii,:)';
    
    [xHat, pHat] = EkfUpdate(xHat, pHat, 0, zi, ti, ...
                             @(x, u, t) x, ...
                             @(x, t) measurementEquation(x, t, qf, qDot, qDDot), V, W);
    
%     msv = max(svd(pHat));
%     fprintf('Max singular value of P: %.6f\n', msv);
%     if msv < 1e-5
%         break
%     end 
    
    if mod(iii, 50) == 0
        PlotCalibrationResults(thetaTruth, xHat, pHat);
    end
end

thetaStar = xHat((end - lengthTheta + 1):end);
thetaStarCov = pHat((end - lengthTheta + 1):end, (end - lengthTheta + 1):end);

end

function y = measurementEquation(theta, t, q, qDot, qDDot)

    [calibBools, numParams, numParamsTotal] = GetCalibInfo();

    % Robot parameters
    x = theta(1:numParams);
    e = zeros(1,numParamsTotal);
    e(calibBools) = e(calibBools) + x';

    % Gravity direction
    gw = theta((numParams + 1):(numParams + 3));

    % System time offset
    tau = theta(numParams + 4);
    tOffset = t + tau;

    qData = q(tOffset);
    qDotData = qDot(tOffset);
    qDDotData = qDDot(tOffset);

    [alph, omeg] = ComputeImuMeasurements(qData, qDotData, qDDotData, e, gw);

    % Data scale factors
    s = theta((numParams + 5):end);
    alph = s(1).*alph;
    omeg = s(2).*omeg;

    y = [alph'; omeg'];
end