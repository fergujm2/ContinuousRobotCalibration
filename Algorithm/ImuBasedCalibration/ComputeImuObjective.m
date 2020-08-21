function res = ComputeImuObjective(theta, t, q, qDot, qDDot, z, cholMeasCovInv)
zModel = ImuMeasurementEquation(theta, t, q, qDot, qDDot);

ez = (zModel - z)';
eMeas = ez(:);

res = cholMeasCovInv*eMeas;

end