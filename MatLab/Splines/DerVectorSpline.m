function [yd, Cd, dd] = DerVectorSpline(y, C, d)
dd = d - 1;

numDimensions = size(C, 1);
n = size(C, 2) - 1;

Cd = zeros(numDimensions, n);

for iii = 1:numDimensions
    [yd, cd] = derspl(d, y, C(iii,:));
    Cd(iii,:) = cd;
end

end