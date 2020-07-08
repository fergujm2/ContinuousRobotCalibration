function [qLimits, qDotLimits, qDDotLimits] = GetJointLimits()

qLimits = [-pi/2, pi/2
               -pi/4, pi/4
               -pi/4, pi/4]; % rad
           
qDotLimits = [-pi/2, pi/2
               -pi/2, pi/2
               -pi/2, pi/2]; % rad/s


qDDotLimits = [-10, 10
               -10, 10
               -10, 10]; % rad/s/s
end