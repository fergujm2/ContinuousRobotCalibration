% Ucubclamp
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate data with a cubic clamped spline

% Define the function to be interpolated
f = @(x) exp(x).*sin(2*pi*x);

% Define its derivative
fd = @(x) exp(x).*(sin(2*pi*x) + 2*pi*cos(2*pi*x));

% Input the number of sample points
m = input('input the number of sample points m =   ');    

% Choose equally sample points in [a,b] and sample f at these points
a = -1; b = 1; t = linspace(a,b,m); z = f(t);

%  Compute the derivatives at a and b
alpha =  fd(a); beta = fd(b);

% Compute the extended knot sequence and coefficient vector
[y,c] = cubclamp(t,z,alpha,beta);

% Set dimension of the spline space
n = m+2;

% Evaluate the spline at ng equally spaced points
ng = 201;  d = 3;
xg = linspace(a,b,ng); 
zg = sval2(d,y,c,xg);
fg = f(xg);

% Plot the spline
figure; plot(xg,zg); hold; plot(xg,fg,'r');

% Compute the max and RMS errors
e = fg-zg;
fprintf('maxerr = %5.2e,  rms = %5.2e\n',norm(e,inf),erms(e));




