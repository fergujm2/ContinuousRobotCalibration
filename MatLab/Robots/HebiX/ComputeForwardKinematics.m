function [p, R, J, frames] = ComputeForwardKinematics(q, e, showPlot)

computeJacobian = nargout >= 3;
returnFrames = nargout >= 4;

numMeas = size(q, 1);

p = zeros(numMeas, 3);
R = zeros(3, 3, numMeas);

if computeJacobian
    J = zeros(6, 3, numMeas);
end

if returnFrames
    frames = zeros(4, 4, 4, numMeas);
end

for iii = 1:numMeas
    if returnFrames
        [p(iii,:), R(:,:,iii), J(:,:,iii), frames(:,:,:,iii)] = computeForwardKinematicsOnce(q(iii,:), e, computeJacobian, returnFrames, showPlot);
    elseif computeJacobian
        [p(iii,:), R(:,:,iii), J(:,:,iii)] = computeForwardKinematicsOnce(q(iii,:), e, computeJacobian, returnFrames, showPlot);
    else
        [p(iii,:), R(:,:,iii)] = computeForwardKinematicsOnce(q(iii,:), e, computeJacobian, returnFrames, showPlot);
    end
end

end

function [p, R, J, frames] = computeForwardKinematicsOnce(q, e, computeJacobian, returnFrames, showPlot)
    dhTable = GetDhTable(q);

    E = reshape(e, 6, 4).';

    E_0 = ErrorTransform(E(1,:), false);
    E_1 = ErrorTransform(E(2,:), false);
    E_2 = ErrorTransform(E(3,:), false);
    E_3 = ErrorTransform(E(4,:), false);

    T_0_1 = DhTransform(dhTable(1,:));
    T_1_2 = DhTransform(dhTable(2,:));
    T_2_3 = DhTransform(dhTable(3,:));

    frames(:,:,1) =                     E_0;
    frames(:,:,2) = frames(:,:,1)*T_0_1*E_1;
    frames(:,:,3) = frames(:,:,2)*T_1_2*E_2;
    frames(:,:,4) = frames(:,:,3)*T_2_3*E_3;

    R = frames(1:3, 1:3, end);
    p = frames(1:3, 4, end);
    
    if computeJacobian
        z1 = frames(1:3,3,1);
        p1 = frames(1:3,4,1);

        z2 = frames(1:3,3,2);
        p2 = frames(1:3,4,2);

        z3 = frames(1:3,3,3);
        p3 = frames(1:3,4,3);

        J1v = cross(z1, p - p1);
        J2v = cross(z2, p - p2);
        J3v = cross(z3, p - p3);

        J = [J1v, J2v, J3v
             z1,  z2,  z3];
    end

    if showPlot
        DrawRobot(frames);
    end
end

