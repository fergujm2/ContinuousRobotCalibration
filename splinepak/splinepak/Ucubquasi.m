% Ucubquasi
% SplinePAK: Copyright Larry Schumaker 2014
% Fit data with a cubic quasi-interpolating spline

% Define the function to be interpolated
f = @(x) exp(x).*sin(2*pi*x);

% Input the number of sample points
m = input('input the number of sample points m =   ');

% Choose equally spaced sample points and evalute f on them
a = -1; b = 1; t = linspace(a,b,m); z = f(t);

% Compute the extended knot sequence and coefficient vector
[y,c] = cubquasi(a,b,z);

% Compute the dimension of  the spline space
n =  m + 2;

% Compute the error 
ng = 100; d = 3;
xg = linspace(a,b,ng);
fg = f(xg);
zg = sval2(d,y,c,xg); e = fg-zg;
fprintf('maxerr = %5.2e,  rms = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline
figure; plot(xg,zg)


