function [pError, rError, res] = ComputeRobotAccuracy(q, e, pMeas, RMeas)
    
    [p, R] = ComputeForwardKinematics(q, e, false);
    
    pError = p - pMeas;
    
    numPts = size(p,1);
    rError = zeros(numPts, 3);

    for iii = 1:numPts
        RError = (R(:,:,iii)')*RMeas(:,:,iii);
        axang = rotm2axang(RError);
        
        ax = axang(1:3);
        ax = ax./norm(ax);
        
        rError(iii,:) = ax*axang(4);
    end
    
    res = [reshape(pError', [], 1); reshape(rError', [], 1)];
end