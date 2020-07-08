function [Ad, Bd] = DerVectorFourier(A, B, T)
numDimensions = size(A, 2);

Ad = zeros(size(A));
Bd = zeros(size(B));

for iii = 1:numDimensions
    [Ad(:,iii), Bd(:,iii)] = DerFourier(A(:,iii), B(:,iii), T);
end

end