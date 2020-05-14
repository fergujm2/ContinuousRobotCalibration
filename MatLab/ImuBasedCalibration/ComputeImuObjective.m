function res = ComputeImuObjective(theta, t, q, qDot, qDDot, z, zCovInv)

alphData = z(:,1:3);
omegData = z(:,4:6);
alphCovInv = zCovInv(1:3,1:3);
omegCovInv = zCovInv(4:6,4:6);

[~, alph, omeg] = ImuMeasurementEquation(theta, t, q, qDot, qDDot);

ealph = (alph - alphData)';
alphres = sqrt(alphCovInv)*ealph;

eomeg = (omeg - omegData)';
omegres = sqrt(omegCovInv)*eomeg;

res = [alphres(:); omegres(:)];

end