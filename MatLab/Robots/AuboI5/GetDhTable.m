function dhTable = GetDhTable(q)

dhTable = [q(1) + 0.0,  0.0985, 0.0,     pi/2
           q(2) + pi/2, 0.1405, 0.4080,  pi
           q(3) + 0.0,  0.1215, 0.3760,  pi
           q(4) + pi/2, 0.1025, 0.0,     pi/2
           q(5) + 0.0,  0.1025, 0.0,    -pi/2
           q(6) + pi,   0.0940, 0.0,     0.0];
end