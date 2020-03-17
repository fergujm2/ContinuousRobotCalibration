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
    Ry = [cos(r(1)), 0, sin(r(1))
          0,         1, 0
         -sin(r(1)), 0, cos(r(1))];

    Rz = [cos(r(2)), -sin(r(2)), 0
          sin(r(2)),  cos(r(2)), 0
          0,                  0, 1];
      
    Rx = [1,  0,      0 
          0,  cos(r(3)), -sin(r(3))
          0,  sin(r(3)), cos(r(3))];

    R = Ry*Rz*Rx;
end

function R = smallAngleRotMat(r)
    R = [1,  -r(2),  r(1)
         r(2),   1, -r(3)
        -r(1),  r(3),   1];
end