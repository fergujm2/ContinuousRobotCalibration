function [y, C] = LsqFitVectorSpline(X, a, b, d, k)
n = k + d + 1; % Dimension of the spline space

% Contruct extended knot vector
y(1:d) = a*ones(1, d);
y((n + 2):(n + d + 1)) = b*ones(1, d);
y((d + 1):(n + 1)) = linspace(a, b, k + 2);

nd = size(X, 1);
t = linspace(a, b, nd);

numDimensions = size(X, 2);

C = zeros(numDimensions, n);

for iii = 1:numDimensions
    z = X(:,iii);
    c = lsqspl(d, n, y, t, z);
    C(iii,:) = c;
end
end