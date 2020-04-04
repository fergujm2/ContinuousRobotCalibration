function [yInt, CInt, dInt] = IntVectorSpline(y, C, d)
dInt = d + 1;

numDimensions = size(C, 1);
n = size(C, 2) + 1;

CInt = zeros(numDimensions, n);

for iii = 1:numDimensions
    c = C(iii,:);
    [cInt, yInt] = intsplco(d, y, c);
    CInt(iii,:) = cInt;
end

end

