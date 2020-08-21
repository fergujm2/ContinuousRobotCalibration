function [x, xDot, xDDot] = GetVectorSplineFunctions(X, t, d, numInteriorKnots)

[y, C] = LsqFitVectorSpline(X, t, d, numInteriorKnots);

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

x = @(t) EvalVectorSpline(y, C, d, t);
xDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
xDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

end