function newx = AnnealingJointLimits(optimValues,problem,T)

while true
    % First get a new point the old way
    newx = annealingboltz(optimValues,problem);

    % Check if the point satisfies the joint limits 
    withinLimits = CheckJointLimits(newx, T);
    
    if withinLimits
        break;
    end
end

end

