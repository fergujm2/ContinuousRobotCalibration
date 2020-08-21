function AB = GetRandomAB(n, numJoints, T)
    mag = 0.1;
%     m = 1./logspace(log10(1), log10(T^(n - 6)), n)';
    m = 1./(1:n)';
%     m = 1./(linspace(log10(10), log10(10000), n))';
%     m = 1./(1:2:2*n)';
    M = mag.*m*ones(1, numJoints);
    
    for iii = 1:1e4
        A = [[0, 0, pi, -pi/3, pi, 0];
            M.*(2.*rand(n, numJoints) - 1)];
        
        B = M.*(2.*rand(n, numJoints) - 1);
        AB = [A; B];
        
        if CheckJointLimits(AB, T)
            return;
        end
    end
    
    error('Could not find random AB satisfying joint limit constraints');
end