clear;

syms('q', [1, 6], 'real');
syms('p', [1, 8*6], 'real');

dhTable = GetDhTable(q);

P = reshape(p, 6, 8).';

E_0 = ErrorTransform(P(1,:), true);
E_1 = ErrorTransform(P(2,:), true);
E_2 = ErrorTransform(P(3,:), true);
E_3 = ErrorTransform(P(4,:), true);
E_4 = ErrorTransform(P(5,:), true);
E_5 = ErrorTransform(P(6,:), true);
E_6 = ErrorTransform(P(7,:), true);
E_7 = ErrorTransform(P(8,:), true);

T_0_1 = DhTransform(dhTable(1,:));
T_1_2 = DhTransform(dhTable(2,:));
T_2_3 = DhTransform(dhTable(3,:));
T_3_4 = DhTransform(dhTable(4,:));
T_4_5 = DhTransform(dhTable(5,:));
T_5_6 = DhTransform(dhTable(6,:));

T_6_7 = [eye(3), [0; 0; 0.12]; [0 0 0 1]];

T =         E_0;
T = simplify(T, 'steps', 10);

T = T*T_0_1*E_1;
T = expand(T);

pTmp = nchoosek(p, 2);
pSub = pTmp(:,1).*pTmp(:,2);

T = subs(T, pSub, zeros(size(pSub)));
T = simplify(T, 'steps', 10);

T = T*T_1_2*E_2;
T = expand(T);
T = subs(T, pSub, zeros(size(pSub)));
T = simplify(T, 'steps', 10);

T = T*T_2_3*E_3;
T = expand(T);
T = subs(T, pSub, zeros(size(pSub)));
T = simplify(T, 'steps', 10);

T = T*T_3_4*E_4;
T = expand(T);
T = subs(T, pSub, zeros(size(pSub)));
T = simplify(T, 'steps', 10);

T = T*T_4_5*E_5;
T = expand(T);
T = subs(T, pSub, zeros(size(pSub)));
T = simplify(T, 'steps', 10);

T = T*T_5_6*E_6;
T = expand(T);
T = subs(T, pSub, zeros(size(pSub)));
T = simplify(T, 'steps', 10);

T = T*T_6_7*E_7;
T = expand(T);
T = subs(T, pSub, zeros(size(pSub)));

t = T(1:3,4);
t = simplify(t, 'steps', 10);

J = jacobian(t, p);
J = simplify(J, 'steps', 10);

disp('Forward Kinematics Formula:');
disp(t);

disp('Identification Jacobian:');
disp(J);

