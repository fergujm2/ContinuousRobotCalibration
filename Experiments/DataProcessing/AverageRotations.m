function qMean = AverageRotations(q)

% This module averages a set of rotations (in SO(3)) by finding the element R*
% that minimizes the sum of distances from R to each of the Ri. This 
% implementation is discussed in F.L. Markley, "Averaging quaternions," 
% Journal of Guidance, Control, and Dynamics vol. 30, no. 4, pp. 1193-1197, 2007.

numMeas = size(q,1);

A = zeros(4);

for iii = 1:numMeas
    A = A + (q(iii,:)')*q(iii,:);
end

A = A./numMeas;    
[V, D] = eig(A);

[~, maxValInd] = max(diag(D));

qMean = V(:,maxValInd)';
end