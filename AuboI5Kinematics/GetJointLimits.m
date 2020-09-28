function [qLimits, qDotLimits, qDDotLimits] = GetJointLimits()

% From AuboI5 data sheet

% qLimits = [-175, 175
%            -175, 175
%            -175, 175
%            -175, 175
%            -175, 175
%            -175, 175].*pi/180; % rad/s
%            
% qDotLimits = [-150, 150
%               -150, 150
%               -150, 150
%               -180, 180
%               -180, 180
%               -180, 180].*pi/180; % rad/s

% Adjustments for safety

qLimits = [-90, 90
           -75, 75
           30, 120
           -175, 175
           -90, 90
           -175, 175].*pi/180; % rad
           
qDotLimits = [-10,  10
              -10,  10
              -10,  10
              -25,  25
              -25,  25
              -25,  25].*pi/180; % rad/s

% qDDotLimits = [-1, 1
%                -1, 1
%                -2, 2
%                -5, 5
%                -5, 5
%                -5, 5]; % rad/s/s
qDDotLimits = 100.*[-1, 1
               -1, 1
               -2, 2
               -5, 5
               -5, 5
               -5, 5]; % rad/s/s
end