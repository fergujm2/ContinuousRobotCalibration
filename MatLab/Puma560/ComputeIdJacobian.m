function J = ComputeIdJacobian(q, calibBools, showPlot)
x = zeros(size(calibBools));

indCalib = find(calibBools);

m = size(q, 1);
l = sum(calibBools);

J = zeros(3*m, l);

del = 1e-16;

indLo = 1;
pts = zeros(3,m);

for iii = 1:m
    indHi = indLo + 2;
    for jjj = 1:l
        p1 = x;
        p2 = x;
        p1(indCalib(jjj)) = p1(indCalib(jjj)) - del;
        p2(indCalib(jjj)) = p2(indCalib(jjj)) + del;
        
        t1 = ComputeForwardKinematics(q(iii,:), p1, false);
        t2 = ComputeForwardKinematics(q(iii,:), p2, false);
        
        J(indLo:indHi,jjj) = (t2 - t1)./2./del;
        
        pts(:,iii) = t1;
    end
    indLo = indLo + 3;
end

if showPlot
    ComputeForwardKinematics(zeros(size(q,2)), x, true);
    plot3(pts(1,:), pts(2,:), pts(3,:), '.k', 'MarkerSize', 10);
end
end