function zCov = ComputeZCov(q, qDot, covBias, kaqd, kwqd)

if nargin == 2
    % These default parameters should be updated with the results of the
    % noise-trajectory dependence script.
    covBias = [0.00186591352878320,0.00128585201828347,0.000960830241355076,3.22586605246488e-06,4.51661957670205e-06,1.97396947043415e-05];
    kaqd = [1.15972268311077;0.273701818754377;0.141837289335669;5.27944486775331;2.22064399355127;2.23871547257960];
    kwqd = [0.0361081620916200;0.0442676701222513;0.0894243932190168;0.0113342408954256;0.0160682527691767;0.0140933599016144];
end

theta = GetThetaNominal();
[x, ~, ~, ~, ra] = UnpackTheta(theta);

[calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
e = zeros(1, numParamsTotal);
e(calibBools) = x;

[~, R, J] = ComputeForwardKinematics(q, e, false);

Ra = eul2rotm(ra');

numPts = size(qDot,1);
zStd = zeros(numPts,6);

for iii = 1:numPts
    aStd = sqrt(covBias(1:3));
    wStd = sqrt(covBias(4:6));

    Ri = (Ra')*(R(:,:,iii)');

    for jjj = 1:size(qDot, 2)
        vj = J(1:3,jjj,iii)*qDot(iii,jjj);
        wj = J(4:6,jjj,iii)*qDot(iii,jjj);

        vjImu = Ri*vj;
        wjImu = Ri*wj;

        aStdQ = kaqd(jjj)*vjImu;
        wStdQ = kwqd(jjj)*wjImu;

        aStd = aStd + abs(aStdQ');
        wStd = wStd + abs(wStdQ');
    end

    zStd(iii,:) = [aStd, wStd];
end

zCov = zStd.^2;
end