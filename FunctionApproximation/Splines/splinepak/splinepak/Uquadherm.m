% Uquadherm
% SplinePAK: Copyright Larry Schumaker 2014
% Use C1 quadratic spline to solve a Hermite interpolation problem

% Define the function to be interpolated
f = @(x) exp(x).*sin(2*pi*x);

% Define its derivative
fd = @(x) exp(x).*(sin(2*pi*x) + 2*pi*cos(2*pi*x));

% Input the number of sample points
m = input('input the number of data points m =  ');

% Choose equally sample points in [0,1] and sample f and fd at these points
a = -1; b = 1; t = linspace(a,b,m);
z = f(t); z1 = fd(t);

% Compute the extended knot sequence and coefficient vector
[y,c] = quadherm(t,z,z1);

% Set dimension of the spline space
n = 2*m;

% Evaluate the spline at ng equally spaced points
ng = 201; d = 2;
xg = linspace(a,b,ng);
zg = sval2(d,y,c,xg);
fg = f(xg);

% Plot the spline
figure; plot(xg,zg); hold; plot(xg,fg,'r');

% Compute the max and RMS errors
e = fg-zg;
fprintf('maxerr = %5.2e,  rms = %5.2e\n',norm(e,inf),erms(e));
