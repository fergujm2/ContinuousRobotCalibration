clear;

syms theta1 theta2
syms gamma0 gamma1 gamma2 a0 a1 a2 b0 b1 b2
syms L1 L2

piFull = [a0, b0, gamma0, a1, b1, gamma1, a2, b2, gamma2];
piCalib = true(1,size(piFull,2));
piCalib(piFull == b1) = false;
piCalib(piFull == b2) = false;

piReduced = piFull(piCalib);

piNominal = [0, 0, 0, L1, 0, 0, L2, 0, 0];

TW1 = [cos(theta1 + gamma0), -sin(theta1 + gamma0), a0
       sin(theta1 + gamma0),  cos(theta1 + gamma0), b0
       0                   ,  0                   , 1];
   
T12 = [cos(theta2 + gamma1), -sin(theta2 + gamma1), a1
       sin(theta2 + gamma1),  cos(theta2 + gamma1), b1
       0                   ,  0                   , 1];
   
T2E = [cos(gamma2), -sin(gamma2), a2
       sin(gamma2),  cos(gamma2), b2
       0          ,  0          , 1];
   
TWE = TW1*T12*T2E;
TWE = simplify(TWE, 'steps', 10);

x = simplify(TWE(1,3), 'steps', 100);
y = simplify(TWE(2,3), 'steps', 100);
z = simplify(acos(TWE(1,1)), 'steps', 100, 'IgnoreAnalyticConstraints', true);

fKin = [x; y; z];

e = simplify(jacobian(fKin, piReduced), 'steps', 10);
e = simplify(subs(e, piFull, piNominal), 'steps', 10);

measurementConfigs = sym('theta_', [5, 2]);
E = getTotalJacobian(e, measurementConfigs);
E = simplify(E, 'steps', 10);

function E = getTotalJacobian(e, measurementConfigs)
    syms theta1 theta2
    
    m = size(measurementConfigs, 1);
    n = size(measurementConfigs, 2);
    
    p = size(e, 2);
    l = size(e, 1);
    
    E = sym(zeros(l*m, p));
    
    indLo = 1;
    
    for iii = 1:m
        indHi = indLo + l - 1;
        ei = subs(e, [theta1, theta2], measurementConfigs(iii,:));
        E(indLo:indHi,:) = ei;
        indLo = indLo + l;
    end
end
