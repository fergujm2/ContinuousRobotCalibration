function robot = SetDhParameters(robot, dhParams)
    
for iii = 1:size(dhParams, 1)
    robot.Bodies{iii}.Joint.setFixedTransform(dhParams(iii,:),'dh');
end

end