function zMeas = ImuMeasurementEquation(theta, t, q, qDot, qDDot)

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();

[x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw] = UnpackTheta(theta);

e = zeros(1,numParamsTotal);
e(calibBools) = e(calibBools) + x';

tOffset = t + tau;

qData = q(tOffset);
qDotData = qDot(tOffset);
qDDotData = qDDot(tOffset);

zModel = ComputeImuMeasurements(qData, qDotData, qDDotData, e, g);

Ta = [1, 0, 0; alphA(1), 1, 0; -alphA(2), alphA(3), 1];
Ka = diag(ka);
Ra = eul2rotm(ra');

Tw = [1, 0, 0; alphW(1), 1, 0; -alphW(2), alphW(3), 1];
Kw = diag(kw);
Rw = eul2rotm(rw');

alph = Ka*Ta*Ra*(zModel(:,1:3)') + ba;
omeg = Kw*Tw*Rw*(zModel(:,4:6)') + bw;

zMeas = [alph', omeg'];
end