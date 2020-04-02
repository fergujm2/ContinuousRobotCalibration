function [p, R] = ComputeForwardKinematics(q, e, showPlot)
numMeas = size(q,1);

p = zeros(numMeas, 3);
R = zeros(3, 3, numMeas);

for iii = 1:numMeas
    [p(iii,:), R(:,:,iii)] = computeForwardKinematicsOnce(q(iii,:), e, showPlot);
end


end

function [p, R] = computeForwardKinematicsOnce(q, e, showPlot)
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

    if showPlot
        DrawPuma(frames);
        draw_coordinate_system(0.125, R, p, ['r' 'g' 'b'], 'E');
    end
end

