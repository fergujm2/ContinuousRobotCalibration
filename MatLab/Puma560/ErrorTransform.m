function E = ErrorTransform(e, smallAngle)
t = e(1:3);
r = e(4:6);

if smallAngle
    R = smallAngleRotMat(r);
else
    R = rotMat(r);
end

E = [R, t'; [0, 0, 0, 1]];
end

function R = rotMat(r)
    a = r(1);
    b = r(2);
    c = r(3);

    Rx = [1,  0,      0 
          0,  cos(a), -sin(a)
          0,  sin(a), cos(a)];

    Ry = [cos(b), 0, sin(b)
          0,      1, 0
         -sin(b), 0, cos(b)];

    Rz = [cos(c), -sin(c), 0
          sin(c),  cos(c), 0
          0,       0,      1];

    R = Rx*Ry*Rz;
end

function R = smallAngleRotMat(r)
    R = [1,    -r(3),  r(2)
         r(3),  1,    -r(1)
        -r(2),  r(1),  1];
end