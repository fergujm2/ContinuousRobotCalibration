function zCov = ComputeZCovPost(t, z, TBin)

d = 5;
[y, C] = LsqFitVectorSpline(z, t, d, floor(length(t)/50));
zf = @(t) EvalVectorSpline(y, C, d, t);

zs = zf(t);
zError = z - zs;

tBin = t(1):TBin:t(end);
zCovBin = zeros(length(tBin) - 1, 6);

for iii = 1:(length(tBin) - 1)
    ind = and(t > tBin(iii), t < tBin(iii + 1));
    zCovBin(iii,:) = diag(cov(zError(ind,:)));
end

tBin = tBin(1:(end - 1)) + TBin/2;

zCov = interp1(tBin, zCovBin, t);

indNan = find(any(isnan(zCov),2));
[~, indChange] = max(diff(indNan));

indLo = indNan(indChange);
indHi = indNan(indChange + 1);

zCovLo = zCov(indLo + 1,:);
zCovHi = zCov(indHi - 1,:);

zCov(1:indLo,:) = repmat(zCovLo, sum(indNan <= indLo), 1);
zCov(indHi:end,:) = repmat(zCovHi, sum(indNan >= indHi), 1);
end