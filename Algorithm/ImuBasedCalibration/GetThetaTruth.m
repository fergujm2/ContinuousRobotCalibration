function theta = GetThetaTruth()

thetaNominal = GetThetaNominal();
[x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw] = UnpackTheta(thetaNominal);

xTruth = x + 0.01.*ones(length(x),1);
gTruth = g(1:2) + [0.1; 0.2];
tauTruth = tau + 0.05;

alphATruth = alphA + [0.01; 0.02; 0.03];
raTruth = ra + [0.02; 0.03; 0.04];
kaTruth = ka + [0.1; 0.2; 0.3];
baTruth = ba + [0.2; 0.3; 0.3];

alphWTruth = alphW + [0.03; 0.04; 0.05];
rwTruth = rw + [0.04; 0.05; 0.06];
kwTruth = kw + [0.3; 0.4; 0.5];
bwTruth = bw + [0.4; 0.5; 0.6];

theta = PackTheta(xTruth, gTruth, tauTruth, alphATruth, raTruth, kaTruth, baTruth, alphWTruth, rwTruth, kwTruth, bwTruth);
end


