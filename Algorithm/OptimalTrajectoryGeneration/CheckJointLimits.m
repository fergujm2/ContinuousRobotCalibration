function withinLimits = CheckJointLimits(AB, T)
    % An AB in the feasible set is one s.t. c < 0. We want to compute the
    % maximum and minumum position velocity and accellerations for each
    % joint and make sure they're all within the correct bounds.
    n = (size(AB,1) - 1)/2;

    A = AB(1:n + 1,:);
    B = AB(n + 2:end,:);
    
    [Ad, Bd] = DerVectorFourier(A, B, T);
    [Add, Bdd] = DerVectorFourier(Ad, Bd, T);
    
    t = linspace(0, T, 1000);
    
    q = EvalVectorFourier(A, B, t, T);
    qDot = EvalVectorFourier(Ad, Bd, t, T);
    qDDot = EvalVectorFourier(Add, Bdd, t, T);
    
    [qLimits, qDotLimits, qDDotLimits] = GetJointLimits();
    
    qLo = qLimits(:,1);
    qHi = qLimits(:,2);
    qDotLo = qDotLimits(:,1);
    qDotHi = qDotLimits(:,2);
    qDDotLo = qDDotLimits(:,1);
    qDDotHi = qDDotLimits(:,2);
    
    % Feasible means that c is negative
    qGood = all(min(q)' > qLo) && all(max(q)' < qHi);
    qDotGood = all(min(qDot)' > qDotLo) && all(max(qDot)' < qDotHi);
    qDDotGood = all(min(qDDot)' > qDDotLo) && all(max(qDDot)' < qDDotHi);
    
    withinLimits = qGood && qDotGood && qDDotGood;
end