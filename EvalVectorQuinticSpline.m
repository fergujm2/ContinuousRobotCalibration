function x = EvalVectorQuinticSpline(y, C, t)
numDimensions = size(C, 1);
numSamples = length(t);

x = zeros(numSamples, numDimensions);

for iii = 1:numDimensions
    c = C(iii,:);
    x(:,iii) = sval2(5, y, c, t);
end

end