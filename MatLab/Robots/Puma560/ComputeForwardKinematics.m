function [p, R, J, frames] = ComputeForwardKinematics(q, e, showPlot)

computeJacobian = nargout >= 3;
returnFrames = nargout >= 4;

numMeas = size(q,1);

p = zeros(numMeas, 3);
R = zeros(3, 3, numMeas);

if computeJacobian
    J = zeros(6, 6, numMeas);
end

if returnFrames
    frames = zeros(4, 4, 8, numMeas);
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

    E = reshape(e, 6, 7).';

    E_0 = ErrorTransform(E(1,:), false);
    E_1 = ErrorTransform(E(2,:), false);
    E_2 = ErrorTransform(E(3,:), false);
    E_3 = ErrorTransform(E(4,:), false);
    E_4 = ErrorTransform(E(5,:), false);
    E_5 = ErrorTransform(E(6,:), false);
    E_6 = ErrorTransform(E(7,:), false);

    T_0_1 = DhTransform(dhTable(1,:));
    T_1_2 = DhTransform(dhTable(2,:));
    T_2_3 = DhTransform(dhTable(3,:));
    T_3_4 = DhTransform(dhTable(4,:));
    T_4_5 = DhTransform(dhTable(5,:));
    T_5_6 = DhTransform(dhTable(6,:));

    T_6_7 = [eye(3), [0; 0; 0.12]; [0 0 0 1]];

    frames(:,:,1) =                     E_0;
    frames(:,:,2) = frames(:,:,1)*T_0_1*E_1;
    frames(:,:,3) = frames(:,:,2)*T_1_2*E_2;
    frames(:,:,4) = frames(:,:,3)*T_2_3*E_3;
    frames(:,:,5) = frames(:,:,4)*T_3_4*E_4;
    frames(:,:,6) = frames(:,:,5)*T_4_5*E_5;
    frames(:,:,7) = frames(:,:,6)*T_5_6*E_6;
    frames(:,:,8) = frames(:,:,7)*T_6_7;

    R = frames(1:3, 1:3, end);
    p = frames(1:3, 4, end);
    
    if computeJacobian
        z1 = frames(1:3,3,1);
        p1 = frames(1:3,4,1);

        z2 = frames(1:3,3,2);
        p2 = frames(1:3,4,2);

        z3 = frames(1:3,3,3);
        p3 = frames(1:3,4,3);

        z4 = frames(1:3,3,4);
        p4 = frames(1:3,4,4);

        z5 = frames(1:3,3,5);
        p5 = frames(1:3,4,5);

        z6 = frames(1:3,3,6);
        p6 = frames(1:3,4,6);

        J1v = cross(z1, p - p1);
        J2v = cross(z2, p - p2);
        J3v = cross(z3, p - p3);
        J4v = cross(z4, p - p4);
        J5v = cross(z5, p - p5);
        J6v = cross(z6, p - p6);

        J = [J1v, J2v, J3v, J4v, J5v, J6v
             z1,  z2,  z3,  z4,  z5,  z6];
    end

    if showPlot
        DrawRobot(frames);
        draw_coordinate_system(0.125, R, p, ['r' 'g' 'b'], 'E');
    end
end

