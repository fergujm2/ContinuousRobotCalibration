syms p1 p2 p3 r1 r2 r3 real

e = [p1, p2, p3, r1, r2, r3];

E = ErrorTransform(e, false);

E = subs(E, [sin(r1), sin(r2), sin(r3)], [r1, r2, r3]);
E = subs(E, [cos(r1), cos(r2), cos(r3)], [1, 1, 1]);
E = subs(E, [r2*r3, r1*r3, r1*r2, r1*r2*r3], [0, 0, 0, 0]);
E = simplify(E, 'steps', 10);

disp("Small Angle Error Transform:");
disp(E);
