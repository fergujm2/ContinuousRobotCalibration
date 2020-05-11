function [y, C] = LsqFitVectorSpline(X, a, b, d, k, lam)
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
    
    if nargin == 6 % i.e. if we want to do penalized least squares
        c = penlsq(d, n, y, t, z, lam);
    else % if not, just do regular least squares
        c = lsqspl(d, n, y, t, z);
    end
    
    C(iii,:) = c;
end
end