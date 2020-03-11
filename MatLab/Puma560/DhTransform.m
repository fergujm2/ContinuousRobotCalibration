function dhT = DhTransform(dhRow)

theta = dhRow(1);
d   = dhRow(2);
a   = dhRow(3);
alpha = dhRow(4);

dhT = [cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)
       sin(theta),  cos(theta)*cos(alpha), -sin(alpha)*cos(theta), a*sin(theta)
       0,           sin(alpha),             cos(alpha),            d
       0,           0,                      0,                     1];
end