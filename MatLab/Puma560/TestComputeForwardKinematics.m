q = zeros(6,1); 
e = zeros(7*6,1); 

for iii = 1:size(e,1)
    for jjj = 1:size(e,2)
        figure(1);
        clf;
        hold on;
        ComputeForwardKinematics(q, e, true);
        ppp = e;
        ppp(iii,jjj) = ppp(iii,jjj) + 0.3;
        ComputeForwardKinematics(q, ppp, true);
        daspect([1,1,1]);
        view([120,30]);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    end
end

