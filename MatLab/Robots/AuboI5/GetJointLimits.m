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

qLimits = [-60, 60
           -60, 60
           30, 120
           -175, 175
           -175, 175
           -175, 175].*pi/180; % rad
           
qDotLimits = [-30,  30
              -30,  30
              -45,  45
              -90,  90
              -90,  90
              -90,  90].*pi/180; % rad/s

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