q = zeros(1, 3); 
e = zeros(4*6, 1); 

for iii = 1:length(q)
    for jjj = 0:0.5:6.28
        qq = q;
        qq(iii) = jjj;
        
        figure(1);
        clf;
        hold on;
        grid on;
        
        ComputeForwardKinematics(qq, e, true);
        
        axis([-4, 4, -4, 4, -2, 3]);
        daspect([1,1,1]);
        view([145, 20]);
        drawnow();
    end
end


for iii = 1:length(e)
    ee = e;
    ee(iii) = ee(iii) + 0.2;

    figure(1);
    clf;
    hold on;
    grid on;

    ComputeForwardKinematics(q, e, true);
    ComputeForwardKinematics(q, ee, true);

    axis([-4, 4, -4, 4, -2, 3]);
    daspect([1,1,1]);
    view([145, 20]);
    drawnow();
end

