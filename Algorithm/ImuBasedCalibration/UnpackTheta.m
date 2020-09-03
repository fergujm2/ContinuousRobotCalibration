function [x, g, tau, alphA, ra, ka, ba, alphW, rw, kw, bw] = UnpackTheta(theta)

[~, numParams] = GetRobotCalibInfo();

x = theta(1:numParams);

gxy = theta((numParams + 1):(numParams + 2));
gz = -sqrt(9.81^2 - sum(gxy.^2));
g = [gxy; gz];

tauInd = numParams + 3;
alphAInd = (numParams + 4):(numParams + 6);
raInd = (numParams + 7):(numParams + 9);
kaInd = (numParams + 10):(numParams + 12);
baInd = (numParams + 13):(numParams + 15);
alphWInd = (numParams + 16):(numParams + 18);
rwInd = (numParams + 19):(numParams + 21);
kwInd = (numParams + 22):(numParams + 24);
bwInd = (numParams + 25):(numParams + 27);

tau = theta(tauInd);
alphA = theta(alphAInd);
ra = theta(raInd);
ka = theta(kaInd);
ba = theta(baInd);
alphW = theta(alphWInd);
rw = theta(rwInd);
kw = theta(kwInd);
bw = theta(bwInd);

end