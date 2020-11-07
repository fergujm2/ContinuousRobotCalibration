function res = ComputeImuObjective(params, tRobot, q, y, d, tImu, z, thetaNominal, Ce)

theta = params(1:length(thetaNominal));
c = params((length(thetaNominal) + 1):end);
C = reshape(c, 6, []);

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

qf = @(t) EvalVectorSpline(y, C, d, t);
qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

qModel = qf(tRobot);
zModel = ImuMeasurementEquation(theta, tImu, qf, qDot, qDDot);

ez = (zModel - z)';
ez = ez(:);

eq = (qModel - q)';
eq = eq(:);

eTheta = theta - thetaNominal;

e = [eq; ez; eTheta];

res = Ce.*e;
end