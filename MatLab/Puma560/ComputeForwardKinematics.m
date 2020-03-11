function [t, R] = ComputeForwardKinematics(q, p, showPlot)

dhTable = GetDhTable(q);

P = reshape(p, 6, 8).';

E_0 = ErrorTransform(P(1,:), false);
E_1 = ErrorTransform(P(2,:), false);
E_2 = ErrorTransform(P(3,:), false);
E_3 = ErrorTransform(P(4,:), false);
E_4 = ErrorTransform(P(5,:), false);
E_5 = ErrorTransform(P(6,:), false);
E_6 = ErrorTransform(P(7,:), false);
E_7 = ErrorTransform(P(8,:), false);

T_0_1 = dhTransform(dhTable(1,:));
T_1_2 = dhTransform(dhTable(2,:));
T_2_3 = dhTransform(dhTable(3,:));
T_3_4 = dhTransform(dhTable(4,:));
T_4_5 = dhTransform(dhTable(5,:));
T_5_6 = dhTransform(dhTable(6,:));

T_6_7 = [eye(3), [0; 0; 0.12]; [0 0 0 1]];

frames(:,:,1) =                     E_0;
frames(:,:,2) = frames(:,:,1)*T_0_1*E_1;
frames(:,:,3) = frames(:,:,2)*T_1_2*E_2;
frames(:,:,4) = frames(:,:,3)*T_2_3*E_3;
frames(:,:,5) = frames(:,:,4)*T_3_4*E_4;
frames(:,:,6) = frames(:,:,5)*T_4_5*E_5;
frames(:,:,7) = frames(:,:,6)*T_5_6*E_6;
frames(:,:,8) = frames(:,:,7)*T_6_7*E_7;

R = frames(1:3, 1:3, end);
t = frames(1:3, 4, end);

if showPlot
    DrawPuma(frames);
    draw_coordinate_system(0.125, R, t, ['r' 'g' 'b'], 'E');
end

end
