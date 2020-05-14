function dhTable = GetDhTable(q)

dhTable = [q(1) - pi/2,  1.0, 0.0, -pi/2
           q(2) + pi,   -1.0, 2.0,  0.0
           q(3),         0.0, 1.5,  0.0];
end