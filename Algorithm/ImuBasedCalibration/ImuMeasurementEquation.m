function z = ImuMeasurementEquation(theta, t, q, qDot, qDDot)

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();

[x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw] = UnpackTheta(theta);

% Robot parameters
e = zeros(1,numParamsTotal);
e(calibBools) = e(calibBools) + x';

tOffset = t + tau;

qData = q(tOffset);
qDotData = qDot(tOffset);
qDDotData = qDDot(tOffset);

z = ComputeImuMeasurements(qData, qDotData, qDDotData, e, g);

Ta = [1, -alphA(1), alphA(2); 0, 1, -alphA(3); 0, 0, 1];
Ka = diag(ka);
Ra = eul2rotm(ra');

Tw = [1, -alphW(1), alphW(2); 0, 1, -alphW(3); 0, 0, 1];
Kw = diag(kw);
Rw = eul2rotm(rw');

alph = z(:,1:3);
omeg = z(:,4:6);

alph = (Ka / Ta)*Ra*alph' + ba;
omeg = (Kw / Tw)*Rw*omeg' + bw;


z = [alph', omeg'];
end