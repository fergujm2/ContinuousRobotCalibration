%  Unotaknotnoise
% SplinePAK: Copyright Larry Schumaker 2014
% Test the cubic not-a-knot and least squares splines with noisy data

% Define the function to be interpolated
f = @(x) (x-.5).^2;

% Input the number of data points
nd = input('input number of data points nd = ');

% Choose equally spaced sample points and evaluate f at these points
a = 0; b = 1; t = linspace(a,b,nd); z = f(t);

% Add noise to the data
eps = input('input eps ');
noise = readnoise(nd);
z = z + eps*noise';

% Compute the not-a-knot interpolant and plot
d = 3; [y,c] = notaknot(d,t,z');  
ng = 100; xg = linspace(a,b,ng);
zg = sval2(d,y,c,xg); fg = f(xg); 
figure; plot(xg,zg); hold; plot(xg,fg,'r');

% Input the number of knots for the least-squares spline
k = input('input the number of knots for least-squares k =  '); 

% Find the dimension of the spline space
n = k + d + 1;

% Set up the extended knot sequence
y(1:d) = a*zeros(1,d); y(n+2:n+d+1) = b*ones(1,d);
y(d+1:n+1) = linspace(a,b,n-d+1);

% Compute the coefficients of lsq spline fit and plot
c = lsqspl(d,n,y,t,z);
zg = sval2(d,y,c,xg);
figure; plot(xg,zg); hold; plot(xg,fg,'r');
fprintf('max error = %g\n',norm(fg-zg,inf));
