function z = ComputeImuMeasurements(q, qDot, qDDot, e, gw)

[~, aAll, omegAll, ~] = ComputeAccellerations(q, qDot, qDDot, e);
alphW = squeeze(aAll(:,end,:))';
omegW = squeeze(omegAll(:,end,:))';

[~, R] = ComputeForwardKinematics(q, e, false);

alph = zeros(size(alphW));
omeg = zeros(size(alphW));

for iii = 1:size(omeg, 1)
    Ri = squeeze(R(:,:,iii));
    
    % Measurement equations
    omeg(iii,:) = (Ri')*(omegW(iii,:)');
    alph(iii,:) = (Ri')*(alphW(iii,:)' - gw);
end

z = [alph, omeg];

end