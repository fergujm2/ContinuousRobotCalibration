d1 =  0.0985;
d2 =  0.1215;
a2 =  0.408;
a3 =  0.376;
d4 =  0.0;
d5 =  0.1025;
d6 =  0.094;

q = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
T = eye(4);

s1 = sin(q(1) + pi); 
c1 = cos(q(1) + pi);
s2 = sin(q(2) - pi/2);
c2 = cos(q(2) - pi/2);
s3 = sin(q(3));
c3 = cos(q(3));
s4 = sin(q(4) - pi/2);
c4 = cos(q(4) - pi/2);
s5 = sin(q(5));
c5 = cos(q(5));
s6 = sin(q(6));
c6 = cos(q(6));

tmp0 = c2*c3;
tmp1 = s2*s3;
tmp2 = c1*tmp0 + c1*tmp1;
tmp3 = c3*s2;
tmp4 = c2*s3;
tmp5 = c1*tmp3 - c1*tmp4;
tmp6 = c4*tmp2 - s4*tmp5;
tmp7 = c5*tmp6 + s1*s5;
tmp8 = -c4*tmp5 - s4*tmp2;
tmp9 = s5*tmp6;
tmp10 = c5*s1;
tmp11 = a2*c2;
tmp12 = s1*tmp0 + s1*tmp1;
tmp13 = s1*tmp3 - s1*tmp4;
tmp14 = c4*tmp12 - s4*tmp13;
tmp15 = -c1*s5 + c5*tmp14;
tmp16 = -c4*tmp13 - s4*tmp12;
tmp17 = c1*c5;
tmp18 = s5*tmp14;
tmp19 = tmp0 + tmp1;
tmp20 = -tmp3 + tmp4;
tmp21 = -c4*tmp19 - s4*tmp20;
tmp22 = c4*tmp20 - s4*tmp19;
tmp23 = c5*tmp22;
tmp24 = s5*tmp22;

T(1,1) = c6*tmp7 + s6*tmp8;
T(1,2) = c6*tmp8 - s6*tmp7;
T(1,3) = -tmp10 + tmp9;
T(1,4) = a3*tmp2 + c1*tmp11 - d2*s1 - d4*s1 + d5*tmp8 - d6*(tmp10 - tmp9);
T(2,1) = c6*tmp15 + s6*tmp16;
T(2,2) = c6*tmp16 - s6*tmp15;
T(2,3) = tmp17 + tmp18;
T(2,4) = a3*tmp12 + c1*d2 + c1*d4 + d5*tmp16 - d6*(-tmp17 - tmp18) + s1*tmp11;
T(3,1) = c6*tmp23 + s6*tmp21;
T(3,2) = c6*tmp21 - s6*tmp23;
T(3,3) = tmp24;
T(3,4) = -a2*s2 + a3*tmp20 + d1 + d5*tmp21 + d6*tmp24;

TCpp = T;

[p, R] = ComputeForwardKinematics(q, zeros(6*7, 1), false);
T = [R, p'; 0, 0, 0, 1];

E = T - TCpp;

fprintf('\nMax difference between cpp and MatLab: %e\n\n', max(max(E)));
