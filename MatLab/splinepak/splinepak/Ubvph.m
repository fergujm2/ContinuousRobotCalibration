% Ubvph
% SplinePAK: Copyright Larry Schumaker 2014
% Solve Lu = (pu')' + qu = f with homogeneous boundary conditions

% Define p, q, f, and the true solution u
p = @(x) exp(x); q = @(x) exp(x);
f = @(x) (1-x).*exp(x+1) -(3+x).*exp(2*x); 
u = @(x) x.*(exp(x)-exp(1));  
a = 0; b = 1;

% Input the degree of the spline and number of knots
d = input('input the degree of the spline d = ');
k = input('input the number of knots k = ');

% Compute the coef vector and extended knot vector of the spline
[c,y] = bvph(d,k,a,b,p,q,f);

% Compare with the true solution
n = k+d+1;  ng = 100;
xg = linspace(a,b,ng); ug = u(xg); zg = sval2(d,y,c,xg);
e = ug - zg;
fprintf('emax = %5.2e, rms = %5.2e\n',norm(e,inf),erms(e));

% Plot the solution
figure; plot(xg,zg);
