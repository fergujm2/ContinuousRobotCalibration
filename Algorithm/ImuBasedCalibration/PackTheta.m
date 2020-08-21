function [theta, thetaCov] = PackTheta(x, g, tau, s, b, xCov, gCov, tauCov, sCov, bCov)

theta = [x; g];
thetaCov = blkdiag(xCov, gCov);

[fitTimeOffset, fitScaleFactors, fitXyzOffsets] = GetCalibOptions();

if fitTimeOffset
    theta = [theta; tau];
    thetaCov = blkdiag(thetaCov, tauCov);
end

if fitScaleFactors
    theta = [theta; s];
    thetaCov = blkdiag(thetaCov, sCov);
end

if fitXyzOffsets
    theta = [theta; b];
    thetaCov = blkdiag(thetaCov, bCov);
end

end