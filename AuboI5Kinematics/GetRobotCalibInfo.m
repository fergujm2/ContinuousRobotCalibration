function [calibBools, numParams, numParamsTotal] = GetRobotCalibInfo()

% Kinematic errors are specified by 6 numbers per frame on the robot.
% In a row, these are: [tx, ty, tz, ry, rz, rx]

% Robot + base calibration assuming position and rotation are measured
% calibBools = logical([1 1 0 1 0 1
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 1 1 1 1]);

% No base calibration (based on Meggiolaro paper)
% calibBools = logical([0 0 0 0 0 0 % None of the base frame params are observable, delete tx0, ty0, ry0, rx0. Still need to remove 1 rotation and 1 translation parameter.
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 1 1 1 1]);

% No base calibration (needed to remove two more unobservable parameters)
% calibBools = logical([0 0 0 0 0 0
%                       1 0 0 0 0 1 % ty1 is in the tz0 direction. Removal of ty1 make total translation parameters removed equal 3. Remove ry1 which is in the rz0 direction makes total rotation parameters removed equal 3.
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 1 1 1 1]);

% Full robot IMU calibration
% calibBools = logical([0 0 0 0 0 0
%                       1 0 0 0 0 1
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 0 1 0 1 
%                       1 1 1 0 0 0]); % Rotation parameters redundant with sensor orientation parameters, so they're removed

% Full robot IMU calibration minus a single link length parameter
calibBools = logical([0 0 0 0 0 0
                      0 0 0 0 0 1
                      0 0 0 1 0 1
                      1 1 0 1 0 1 
                      1 1 0 1 0 1 
                      1 1 0 1 0 1 
                      1 1 1 0 0 0]);

% % Robot IMU calibration, only rotation robot parameters considered
% calibBools = logical([0 0 0 0 0 0 
%                       0 0 0 0 0 1 % all t's removed
%                       0 0 0 1 0 1 % all t's removed
%                       0 0 0 1 0 1 % all t's removed
%                       0 0 0 1 0 1 % all t's removed
%                       0 0 0 1 0 1 % all t's removed
%                       1 1 1 0 0 0]);

% Only IMU translation offset considered
% calibBools = logical([0 0 0 0 0 0 
%                       0 0 0 0 0 0 % all r's removed
%                       0 0 0 0 0 0 % all r's removed
%                       0 0 0 0 0 0 % all r's removed
%                       0 0 0 0 0 0 % all r's removed
%                       0 0 0 0 0 0 % all r's removed
%                       1 1 1 0 0 0]);

calibBools = reshape(calibBools', 1, []);
numParamsTotal = length(calibBools);
numParams = sum(calibBools);

end