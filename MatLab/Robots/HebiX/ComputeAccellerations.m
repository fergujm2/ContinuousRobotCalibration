function [v, a, omeg, alph] = ComputeAccellerations(q, qDot, qDDot, e)
    N = size(q, 1);
    
    [~, ~, ~, frames] = ComputeForwardKinematics(q, e, false);
    
    v = zeros(3, 4, N);
    a = zeros(3, 4, N);
    omeg = zeros(3, 4, N);
    alph = zeros(3, 4, N);
    
    for iii = 1:N
        [v(:,:,iii), a(:,:,iii), omeg(:,:,iii), alph(:,:,iii)] = computeAccellerationsOnce(frames(:,:,:,iii), qDot(iii,:), qDDot(iii,:));
    end
end

function [v, a, omeg, alph] = computeAccellerationsOnce(frames, qDot, qDDot)
    qDot(end + 1) = 0;
    qDDot(end + 1) = 0;
    
    z = squeeze(frames(1:3, 3, :));
    p = squeeze(frames(1:3, 4, :));
    r = diff(p, 1, 2);
    
    v = zeros(3, 4);
    a = zeros(3, 4);
    omeg = zeros(3, 4);
    alph = zeros(3, 4);
    
    % We can do this with this recurrence relationship
    for iii = 1:3
        [v(:,iii + 1), a(:,iii + 1), omeg(:,iii + 1), alph(:,iii + 1)] = newtonRecurrence(v(:,iii), a(:,iii), omeg(:,iii), alph(:,iii), z(:,iii), r(:,iii), qDot(iii), qDDot(iii));
    end
end

function [vb, ab, omegb, alphb] = newtonRecurrence(va, aa, omega, alpha, za, rb, qDot, qDDot)
    omegb = omega + za*qDot;
    alphb = alpha + za*qDDot + cross(omega, za*qDot);
    vb = va + cross(omegb, rb);
    ab = aa + cross(alphb, rb) + cross(omegb, cross(omegb, rb));
end