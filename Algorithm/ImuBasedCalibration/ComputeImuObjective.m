function res = ComputeImuObjective(theta, q, qDot, qDDot, tImu, z, sqrtMeasCovInv)

zModel = ImuMeasurementEquation(theta, tImu, q, qDot, qDDot);

ez = (zModel - z)';
eMeas = ez(:);

res = sqrtMeasCovInv*eMeas;
end