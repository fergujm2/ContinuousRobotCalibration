function dhTable = GetDhParameters(robot)

dhTable = zeros(robot.NumBodies, 4);
    
for iii = 1:size(dhTable, 1)
    TDh = robot.Bodies{iii}.Joint.ChildToJointTransform;
    dhTable(iii,:) = getDhRow(TDh);
end

end

function dhRow = getDhRow(TDh)
    d = TDh(3,4);
    
    sina = TDh(3,2);
    cosa = TDh(3,3);
    alpha = atan2(sina, cosa);
    
    cost = TDh(1,1);
    sint = TDh(2,1);
    theta = atan2(sint, cost);
    
    T14 = TDh(1,4);
    T24 = TDh(2,4);
    
    if abs(T14) < 1e-10 && abs(T24) < 1e-10
        a = 0;
    else
        a = (T14 + T24)/(cost + sint);
    end
    
    dhRow = [a, alpha, d, theta];
end