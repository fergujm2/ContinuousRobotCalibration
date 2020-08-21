q = zeros(1, 6); 
e = zeros(7*6, 1);

for iii = 1:length(q)
    for jjj = 0:0.5:6.28
        qq = q;
        qq(iii) = jjj;
        
        figure(1);
        clf;
        hold on;
        grid on;
        axis([-1/2, 1/2, -1/2, 1/2, 0, 2]);
        axis manual;
        daspect([1,1,1]);
        view([145, 20]);
        
        ComputeForwardKinematics(qq, e, true);

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
    axis([-1/2, 1/2, -1/2, 1/2, 0, 2]);
    axis manual;
    daspect([1,1,1]);
    view([145, 20]);

    ComputeForwardKinematics(q, e, true);
    ComputeForwardKinematics(q, ee, true);

    drawnow();
end

