function [q, qDot, qDDot] = FitJointValueFunctions(q, tSpan, numInteriorKnots, lam)

d = 5;

if nargin == 2 % Regular interpolation
    [y, C] = FitVectorSpline(q, tSpan(1), tSpan(2), d);
elseif nargin == 3 % Least squares interpolation
    [y, C] = LsqFitVectorSpline(q, tSpan(1), tSpan(2), d, numInteriorKnots);
elseif nargin == 4 % Penalized least squares with parameter lam
    [y, C] = LsqFitVectorSpline(q, tSpan(1), tSpan(2), d, numInteriorKnots, lam);
end

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

q = @(t) EvalVectorSpline(y, C, d, t);
qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);

end