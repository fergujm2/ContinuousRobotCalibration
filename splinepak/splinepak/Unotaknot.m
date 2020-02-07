% Unotaknot
% SplinePAK: Copyright Larry Schumaker 2014
% Perform not-a-knot interpolation

% Define the function to be interpolated
%f = @(x) 1./(1+25*x.^2);
f = @(x) exp(x).*sin(2*pi*x);

% Input the degree of the spline and number of sample points
d = input('input the degree of the spline (Odd) d =   ');
n = input('input the number of data points n =   ');

% Set up n equally-spaced sample points and evaluate f at these points
a = -1; b = 1; t = linspace(a,b,n);  z = f(t)';

% Call notaknot to get extended knot sequence y and coefficient vector c
[y,c] = notaknot(d,t,z);

% Evaluate the spline on a set of ng equally spaced points
ng = 201;  xg = linspace(y(1),y(n+1),ng);
zg = sval2(d,y,c,xg);
fg = f(xg);

% Plot the spline
figure; plot(xg,zg); hold; plot(xg,fg,'r');

%Compute the max and RMS errors
e = fg - zg;
fprintf('maxerr = %5.2e,  rms = %5.2e\n',norm(e,inf),erms(e));
