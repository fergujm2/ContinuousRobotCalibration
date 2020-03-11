syms e1 e2 e3 e4 e5 e6 real

e = [e1, e2, e3, e4, e5, e6];

E = ErrorTransform(e, false);

E = subs(E, [sin(e4), sin(e5), sin(e6)], [e4, e5, e6]);
E = subs(E, [cos(e4), cos(e5), cos(e6)], [1, 1, 1]);
E = subs(E, [e5*e6, e4*e6, e4*e5, e4*e5*e6], [0, 0, 0, 0]);
E = simplify(E, 'steps', 10);

pretty(E);
