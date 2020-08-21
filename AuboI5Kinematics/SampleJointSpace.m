function q = SampleJointSpace(numMeas)

jointLimits = GetJointLimits();

numJoints = size(jointLimits, 1);
q = zeros(numMeas,numJoints);
            
for iii = 1:numJoints
    qLo = jointLimits(iii,1);
    qHi = jointLimits(iii,2);
    
    q(:,iii) = (qHi - qLo)*rand(1,numMeas) + qLo;
end

end