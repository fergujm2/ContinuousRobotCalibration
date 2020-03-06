function p = ComputeForwardKinematics(robot, q, varargin)

if nargin > 2
    ip = inputParser();
    addRequired(ip, 'x');
    addRequired(ip, 'dhParamsNominal');
    addRequired(ip, 'calibBools');

    parse(ip, varargin{:});
    
    dhParams = ip.Results.dhParamsNominal;
    dhParams(ip.Results.calibBools) = dhParams(ip.Results.calibBools) + ip.Results.x;
    
    robot = SetDhParameters(robot, dhParams);
end


numMeas = size(q, 1);
p = zeros(numMeas, 3);
    
homeConfig = robot.homeConfiguration();
numJoints = length(homeConfig);
    
for iii = 1:numMeas
    qConfig = homeConfig;
       
    for jjj = 1:numJoints
        qConfig(jjj).JointPosition = q(iii,jjj);
    end
    Ti = robot.getTransform(qConfig, robot.BodyNames{end});
    p(iii,:) = Ti(1:3,4);
end

end