%% Setup

clear;

syms('q_', [1, 6], 'real');
syms('e_', [7, 6], 'real');
e = e';
e = e(:);

%% Forward Kinematics and ID Jacobian

dhTable = GetDhTable(q);

E = reshape(e, 6, 7).';

E_0 = ErrorTransform(E(1,:), true);
E_1 = ErrorTransform(E(2,:), true);
E_2 = ErrorTransform(E(3,:), true);
E_3 = ErrorTransform(E(4,:), true);
E_4 = ErrorTransform(E(5,:), true);
E_5 = ErrorTransform(E(6,:), true);
E_6 = ErrorTransform(E(7,:), true);

T_0_1 = DhTransform(dhTable(1,:));
T_1_2 = DhTransform(dhTable(2,:));
T_2_3 = DhTransform(dhTable(3,:));
T_3_4 = DhTransform(dhTable(4,:));
T_4_5 = DhTransform(dhTable(5,:));
T_5_6 = DhTransform(dhTable(6,:));

T_6_7 = [eye(3), [0; 0; 0.12]; [0 0 0 1]];

T = E_0;
T = simplify(T, 'steps', 10);

T = T*T_0_1*E_1;
T = expand(T);

eTmp = nchoosek(e, 2);
eSub = eTmp(:,1).*eTmp(:,2);

T = subs(T, eSub, zeros(size(eSub)));
T = simplify(T, 'steps', 10);

T = T*T_1_2*E_2;
T = expand(T);
T = subs(T, eSub, zeros(size(eSub)));
T = simplify(T, 'steps', 10);

T = T*T_2_3*E_3;
T = expand(T);
T = subs(T, eSub, zeros(size(eSub)));
T = simplify(T, 'steps', 10);

T = T*T_3_4*E_4;
T = expand(T);
T = subs(T, eSub, zeros(size(eSub)));
T = simplify(T, 'steps', 10);

T = T*T_4_5*E_5;
T = expand(T);
T = subs(T, eSub, zeros(size(eSub)));
T = simplify(T, 'steps', 10);

T = T*T_5_6*E_6;
T = expand(T);
T = subs(T, eSub, zeros(size(eSub)));
T = simplify(T, 'steps', 10);

T = T*T_6_7;
T = expand(T);
T = subs(T, eSub, zeros(size(eSub)));

t = T(1:3,4);
t = simplify(t, 'steps', 10);

J = jacobian(t, e);
J = simplify(J, 'steps', 10);

disp('Forward Kinematics Formula:');
disp(t);

disp('Identification Jacobian:');
disp(J);

%% Analyze the Columns of the ID Jacobian

% We have this many generalized error parameters
N = 6*(6 + 1);

% Number of linear dependencies
% a4, a5, a6 = 0 => 6 - q + 1 = 4 =>
q = 1;
k = 3 + 2*q;
D = 2*6 + 4*0 + k;

% Number of linearly independent parameters
I = N - D;

a = dhTable(:,end - 1);
alpha = dhTable(:,end);

calibBools = true(size(e));

for iii = 1:6
    im13 = 6*(iii - 1) + 3;
    im15 = 6*(iii - 1) + 5;
    
    i2 = 6*iii + 2;
    i3 = 6*iii + 3;
    i4 = 6*iii + 4;
    i5 = 6*iii + 5;
    
    sai = sin(alpha(iii));
    cai = cos(alpha(iii));
    
    lc1 = J(:,im13) - sai*J(:,i2) - cai*J(:,i3);
    lc2 = J(:,im15) - a(iii)*cai*J(:,i2) + a(iii)*sai*J(:,i3) - sai*J(:,i4) - cai*J(:,i5);
    
    lc1 = simplify(lc1, 'steps', 10);
    lc2 = simplify(lc2, 'steps', 10);
    
    if all(lc1 == 0) 
        calibBools(im13) = 0;
    else
        warning("Linear combination not equal to zero.");
    end
    
    if all(lc2 == 0)
        calibBools(im15) = 0;
    else
        warning("Linear combination not equal to zero.");
    end
end

% There are going to also be zero columns for the rotational measurements
in4 = 6*6 + 4;
in5 = 6*6 + 5;
in6 = 6*6 + 6;

calibBools([in4, in5, in6]) = 0;

% Also check last few cols
nm14 = 6*(6 - 1) + 4;
nm11 = 6*(6 - 1) + 1;

null(J(:,[nm14, nm11]));

nm16 = 6*(6 - 1) + 6;
nm12 = 6*(6 - 1) + 2;

null(J(:,[nm16, nm12]));

nm15 = 6*(6 - 1) + 5;

J(:,nm15);

calibBools([nm14, nm16, nm15]) = 0;