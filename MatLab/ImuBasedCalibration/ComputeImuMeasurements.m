function [alph, omeg] = ComputeImuMeasurements(q, t, e, gw)
del = 1e-3;

tLeft = t - del;
tRight = t + del;

[pLeft, RLeft] = ComputeForwardKinematics(q(tLeft), e, false);
[p, R] = ComputeForwardKinematics(q(t), e, false);
[pRight, RRight] = ComputeForwardKinematics(q(tRight), e, false);

% Second order central difference
alphW = (pRight - 2*p + pLeft)./(del^2);

% First order central difference
RDot = (RRight - RLeft)./(2*del);

alph = zeros(size(alphW));
omeg = zeros(size(alphW));

for iii = 1:size(omeg, 1)
    % Determine omega in world frame from RDot and R
    Ri = squeeze(R(:,:,iii));
    RDoti = squeeze(RDot(:,:,iii));
    Omeg = RDoti*(Ri');
    omegW = [Omeg(3,2), Omeg(1,3), Omeg(2,1)];
    
    % Measurement equations
    omeg(iii,:) = (Ri')*(omegW');
    alph(iii,:) = (Ri')*(alphW(iii,:)' - gw);
end

end