% Ubvp
% SplinePAK: Copyright Larry Schumaker 2014
% Solve Lu = -(pu')' + qu = f with given boundary values

% Define p,p',q,f, and the true solution u
p = @(x) exp(x);
pp = @(x) exp(x);
q = @(x) sin(x^2);
pi3 = 3*pi;
f = @(x) (1+x)*sin(x^2) -exp(x)  -pi3*exp(x)*(2+x)*cos(pi3*x) ...
      + sin(pi3*x)*(exp(x)*(9*pi^2*x-1) + x*sin(x^2));
u = @(x) 1 + x + x.*sin(pi3.*x);

% set the boundary values 
a = 0; b = 1;
alpha = 1; beta = 2;

% Input the degree of the spline and number of knots
d = input('input the degree of the spline d = ');
k = input('input the number of knots k = ');

% Find the coefficients and extended knot sequence of the spline
if d > 1 
  [c,y,A] = bvp(d,k,a,b,p,pp,q,f,alpha,beta);
  fprintf('Condition number %5.2e \n', 1/rcond(A));
else
  [c,y,A] = bvp1(k,a,b,p,pp,q,f,alpha,beta);
   fprintf('Condition number %5.2e \n', 1/rcond(A));
end

% Compare with true solution
n = k+d+1;  ng = 100;
xg = linspace(a,b,ng); ug = u(xg);
zg = sval2(d,y,c,xg);
e = ug - zg;
fprintf('emax = %5.2e, rms = %5.2e\n',norm(e,inf),erms(e));

% Plot solution
figure; plot(xg,zg);
