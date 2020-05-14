function [x, g, tau, s, xCov, gCov, tauCov, sCov] = UnpackTheta(theta, thetaCov)

[~, numParams] = GetRobotCalibInfo();
[fitTimeOffset, fitScaleFactors] = GetCalibOptions();

x = theta(1:numParams);
g = theta((numParams + 1):(numParams + 3));

if fitTimeOffset
    tau = theta(numParams + 4);
else
    tau = nan;
end

if fitScaleFactors
    s = theta((numParams + 5):end);
else
    s = nan(2,1);
end

if nargin == 2
    xCov = thetaCov(1:numParams,1:numParams);
    gCov = thetaCov((numParams + 1):(numParams + 3),(numParams + 1):(numParams + 3));
    
    if fitTimeOffset
        tauCov = thetaCov((numParams + 4),(numParams + 4));
    else
        tauCov = nan;
    end
    
    if fitScaleFactors
        sCov = thetaCov((numParams + 5):end,(numParams + 5):end);
    else
        sCov = nans(2);
    end
end
end