function [withinLimits, nonlinearConstraint] = CheckJointLimits(q, qDot, qDDot, tSpan, jointLimitTol)
    % An AB in the feasible set is one s.t. c < 0. We want to compute the
    % maximum and minumum position velocity and accellerations for each
    % joint and make sure they're all within the correct bounds.
    
    [qLimits, qDotLimits, qDDotLimits] = GetJointLimits();
    
    qLo = qLimits(:,1);
    qHi = qLimits(:,2);
    qDotLo = qDotLimits(:,1);
    qDotHi = qDotLimits(:,2);
    qDDotLo = qDDotLimits(:,1);
    qDDotHi = qDDotLimits(:,2);
    
    t = linspace(tSpan(1), tSpan(2), (tSpan(2) - tSpan(1))*100);
    q = q(t);
    qDot = qDot(t);
    qDDot = qDDot(t);
    
    % Constraint is that qMin >= qLo, qMax <= qHi.
    % MatLab needs <= only, so 
    cq = [max(q)' - qHi; -(min(q)' - qLo)];
    cqDot = [max(qDot)' - qDotHi; -(min(qDot)' - qDotLo)];
    cqDDot = [max(qDDot)' - qDDotHi; -(min(qDDot)' - qDDotLo)];
    
    nonlinearConstraint = [cq; cqDot; cqDDot];
    
    if nargin < 5
        jointLimitTol = 0;
    end

    withinLimits = all(nonlinearConstraint <= jointLimitTol);
    
%     fprintf('\nwithinLimits: %s', mat2str(withinLimits));
end