function [theta, thetaCov] = PackTheta(x, g, tau, s, xCov, gCov, tauCov, sCov)

theta = [x; g];

[fitTimeOffset, fitScaleFactors] = GetCalibOptions();

if fitTimeOffset
    theta = [theta; tau];
end

if fitScaleFactors
    theta = [theta; s];
end

if nargout == 2
    thetaCov = blkdiag(xCov, gCov);
    
    if fitTimeOffset
        thetaCov = blkdiag(thetaCov, tauCov);
    end
    
    if fitScaleFactors
        thetaCov = blkdiag(thetaCov, sCov);
    end
end
end