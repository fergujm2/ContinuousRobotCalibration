function [A, B] = FitVectorFourier(X, a, b, T, n)
N = size(X, 1);
t = mod(linspace(a, b, N), T);

numDimensions = size(X, 2);

A = zeros(n + 1, numDimensions);
B = zeros(n, numDimensions);

for iii = 1:numDimensions
    [A(:,iii), B(:,iii)] = Fseries(t, X(:,iii)', n);
end

end