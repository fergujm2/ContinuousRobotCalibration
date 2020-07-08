function X = EvalVectorFourier(A, B, t, T)
numDimensions = size(A, 2);
numSamples = length(t);

X = zeros(numSamples, numDimensions);

for iii = 1:numDimensions
    X(:,iii) = Fseriesval(A(:,iii), B(:,iii), mod(t, T));
end

end