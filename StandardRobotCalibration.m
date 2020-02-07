clear;

robot = GetRobot();

dhParamsNominal = GetDhParameters(robot);

calibBools = false(size(dhParamsNominal));

calibBools(1,2) = true;
calibBools(3,1) = true;
calibBools(5,3) = true;

dhParamsTruth = dhParamsNominal;

xCov = diag([0.1, 0.2, 0.3]);
xTruth = mvnrnd(zeros(3,1), xCov, 1)';

dhParamsTruth(calibBools) = dhParamsNominal(calibBools) + xTruth;

% Measure robot position
numMeas = 20;
pCovMag = 0.001^2;
pCovSingleMeas = pCovMag*eye(3);
pCov = pCovMag*eye(3*numMeas);
qCov = zeros(6);
showPlot = false;

robot = SetDhParameters(robot, dhParamsTruth);
[p, q] = GenerateDiscreteMeasurements(robot, numMeas, 'pCov', pCovSingleMeas, 'qCov', qCov, 'ShowPlot', showPlot);
robot = SetDhParameters(robot, dhParamsNominal);

numVars = sum(sum(calibBools));

% Regular least-squares estimate

x0 = zeros(numVars, 1);
H = computeJacobian(x0, robot, p, q, dhParamsNominal, calibBools);
Z = computeResidual(x0, robot, p, q, dhParamsNominal, calibBools);

xStarLsq = -((H')*H) \ (H')*Z;

% Minimum-variance estimate

xStarMinVar = -(inv((inv(xCov) + (H')*(inv(pCov))*H)))*(H')*(inv(pCov))*Z;

% Nonlinear least-squares estimate

obj = @(x) computeResidual(x, robot, p, q, dhParamsNominal, calibBools);
options = optimoptions(@lsqnonlin, ...
                       'Algorithm','levenberg-marquardt', ...
                       'Display', 'iter');
                   
xStarNlLsq = lsqnonlin(obj, x0, [], [], options);


function jac = computeJacobian(x, robot, p, q, dhParamsNominal, calibBools)
    del = 1e-6;
    
    numVars = length(x);
    numMeas = size(p, 1);
    jac = zeros(3*numMeas, numVars);
    
    for iii = 1:numVars
        x1 = x;
        x2 = x;
        
        x2(iii) = x2(iii) + del;
        x1(iii) = x1(iii) - del;
        
        res2 = computeResidual(x2, robot, p, q, dhParamsNominal, calibBools);
        res1 = computeResidual(x1, robot, p, q, dhParamsNominal, calibBools);
        
        jac(:,iii) = (res2 - res1)./(2*del);
    end
end

function res = computeResidual(x, robot, p, q, dhParamsNominal, calibBools)
    dhParams = dhParamsNominal;
    dhParams(calibBools) = dhParams(calibBools) + x;
    
    robot = SetDhParameters(robot, dhParams);
    
    numMeas = size(p, 1);
    
    resMatrix = zeros(3, numMeas);
    
    homeConfig = robot.homeConfiguration();
    numJoints = length(homeConfig);
    
    for iii = 1:numMeas
        qConfig = homeConfig;
        
        for jjj = 1:numJoints
            qConfig(jjj).JointPosition = q(iii,jjj);
        end
        Ti = robot.getTransform(qConfig, robot.BodyNames{end});
        resMatrix(:,iii) = p(iii,:)' - Ti(1:3,4);
    end
    res = resMatrix(:);
end