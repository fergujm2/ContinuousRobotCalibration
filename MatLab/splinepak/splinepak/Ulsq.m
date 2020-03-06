% Ulsq
% SplinePAK: Copyright Larry Schumaker 2014
% Compute the discrete least squares fit of given data
%   using a spline of degree d with given knots

% Define the function to be interpolated
f = @(x) exp(x).*sin(2*pi*x);

% Input the degree of the spline and number of knots
d = input('input the degree of the spline d = ');
k = input('input the number of knots k = '); 

% Find dimension of the spline space
n = k+d+1;

% Set up the extended knot sequence
a = -1; b = 1;
y(1:d) = a*ones(1,d); y(n+2:n+d+1) = b*ones(1,d);
y(d+1:n+1) = linspace(a,b,k+2);

% Input the number of data points
nd = input('input nd ');

% Choose equally spaced sample points and evaluate f at these points
t = linspace(a,b,nd); z = f(t);

% Compute the coefficients of the lsq spline fit
c = lsqspl(d,n,y,t,z);

% Compute the error 
ng = 201; xg = linspace(a,b,ng);
fg = f(xg); zg = sval2(d,y,c,xg); e = fg-zg;
fprintf('maxerr = %5.2e,  rms = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline
figure; plot(xg,zg); hold; plot(xg,fg,'r');

