function [x, g, tau, s, b, xCov, gCov, tauCov, sCov, bCov] = UnpackTheta(theta, thetaCov)

unpackCov = nargin == 2;

[~, numParams] = GetRobotCalibInfo();
[fitTimeOffset, fitScaleFactors, fitXyzOffsets] = GetCalibOptions();

x = theta(1:numParams);
gxy = theta((numParams + 1):(numParams + 2));
gz = -sqrt(9.81^2 - sum(gxy.^2));
g = [gxy; gz];

if unpackCov
    xCov = thetaCov(1:numParams,1:numParams);
    gCov = thetaCov((numParams + 1):(numParams + 2),(numParams + 1):(numParams + 2));
end

thetaInd = numParams + 3;

if fitTimeOffset
    tau = theta(thetaInd);
    if unpackCov
        tauCov = thetaCov(thetaInd,thetaInd);
    end
    thetaInd = thetaInd + 1;
else
    tau = nan;
    tauCov = nan;
end

if fitScaleFactors
    s = theta(thetaInd:(thetaInd + 1));
    if unpackCov
        sCov = thetaCov(thetaInd:(thetaInd + 1),thetaInd:(thetaInd + 1));
    end
    thetaInd = thetaInd + 2;
else
    s = nan(2,1);
    sCov = nan(2);
end

if fitXyzOffsets
    b = theta(thetaInd:end);
    if unpackCov
        bCov = thetaCov(thetaInd:end,thetaInd:end);
    end
else
    b = nan(6,1);
    bCov = nan(6);
end
end