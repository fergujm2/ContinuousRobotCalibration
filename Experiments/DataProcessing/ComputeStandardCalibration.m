function e = ComputeStandardCalibration(q, p, R)

    calibBools = logical([1 1 0 1 0 1
                          1 1 0 1 0 1 
                          1 1 0 1 0 1 
                          1 1 0 1 0 1 
                          1 1 0 1 0 1 
                          1 1 0 1 0 1 
                          1 1 1 1 1 1]);

    calibBools = reshape(calibBools', 1, []);
    
    obj = @(x) computeResidual(x, q, p, R, calibBools);
    
    % Get a good guess for the base frame
    x0 = zeros(sum(calibBools), 1);
    options = optimoptions(@lsqnonlin, ...
                           'Algorithm', 'levenberg-marquardt', ...
                           'Display', 'iter', ...
                           'StepTolerance', 1e-8, ...
                           'FunctionTolerance', 1e-8);
                       
    x = lsqnonlin(obj, x0, [], [], options);
    
    e = zeros(length(calibBools),1);
    e(calibBools) = x;
end

function res = computeResidual(x, q, p, R, calibBools)

e = zeros(1, length(calibBools));
e(calibBools) = x;

[~, ~, res] = ComputeRobotAccuracy(q, e, p, R);

end