function DrawRobot(frames)
T0 = frames(:,:,1);
T1 = frames(:,:,2);
T2 = frames(:,:,3);
T3 = frames(:,:,4);

draw_coordinate_system(0.5, T0(1:3,1:3), T0(1:3,4), ['r' 'g' 'b'], 'B');
draw_coordinate_system(0.5, T1(1:3,1:3), T1(1:3,4), ['r' 'g' 'b'], '1');
draw_coordinate_system(0.5, T2(1:3,1:3), T2(1:3,4), ['r' 'g' 'b'], '2');
draw_coordinate_system(0.5, T3(1:3,1:3), T3(1:3,4), ['r' 'g' 'b'], '3');

end
