
syms tx ty tz ry rz rx;


Ry = [cos(ry), 0, sin(ry)
      0,       1, 0
     -sin(ry), 0, cos(ry)];

Rz = [cos(rz), -sin(rz), 0
      sin(rz),  cos(rz), 0
      0,              0, 1];

Rx = [1,  0,      0 
      0,  cos(rx), -sin(rx)
      0,  sin(rx), cos(rx)];

R = Ry*Rz*Rx;

Er = [R, zeros(3,1); [0, 0, 0, 1]];
Et = [eye(3), [tx; ty; tz]; [0, 0, 0, 1]];

E = Et*Er;

E = simplify(expand(E), 'steps', 100);