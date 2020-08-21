function [y, C] = FitVectorSpline(X, a, b, d)
n = size(X,1);
t = linspace(a, b, n);

numDimensions = size(X,2);

C = zeros(numDimensions, n);

for iii = 1:numDimensions
    z = X(:,iii);
    [y, c] = notaknot(d, t, z);
    C(iii,:) = c;
end
end