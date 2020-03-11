q = zeros(6,1); 
p = zeros(8*6,1); 

for iii = 1:size(p,1)
    for jjj = 1:size(p,2)
        figure(1);
        clf;
        hold on;
        ComputeForwardKinematics(q, p, true);
        ppp = p;
        ppp(iii,jjj) = ppp(iii,jjj) + 0.3;
        ComputeForwardKinematics(q, ppp, true);
        daspect([1,1,1]);
        view([120,30]);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    end
end

