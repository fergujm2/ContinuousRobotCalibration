% Uknotrem
% SplinePAK: Copyright Larry Schumaker 2014
% Do least squares spline fitting with a given number of knots
%   Uses knot removal to choose the knot locations

% Define the function to be fit
f = @(x) (0<= x & x < .5).*(1-2*x) + (.5<= x & x <= 1).*(x-.5);

% Input the degree of the spline and number of initial knots
d = input('input the degree of the spline d = '); m = d+1;
k0 = input('input the initial number of knots k0 =  ');

% Find the dimension of the spline space
n = k0 + d + 1; 

% Find the extended knot sequence
y0(1:d) = zeros(1,d); y0(n+2:n+d+1) = ones(1,d);
y0(d+1:n+1) = linspace(0,1,n-d+1);

% Set the number of data points
nd = 201;

% Choose equally spaced sample points and evaluate f at these points
t = linspace(0,1,nd);
z = f(t);

% Compute the coefficients of the lsq spline fit
c = lsqspl(d,n,y0,t,z);

% Compute errors and plot
ng = 201;
xg = linspace(0,1,ng);
fg = f(xg);
zg = sval2(d,y0,c,xg); e = fg - zg;
fprintf('emax = %5.2e, rms = %5.2e\n',norm(e,inf),erms(e));
figure; plot(xg,zg);

% Perform knot removal with a goal of getting down to k knots
k = input('input the desired number of knots k =  ');
tic
[c,y] = knotrem(d,k,k0,y0,xg,zg);
toc

%Compute errors and plot
n = k + 4;
zg = sval2(d,y,c,xg); e = fg-zg;
fprintf('emax = %5.2e, rms = %5.2e\n',norm(e,inf),erms(e));
figure; plot(xg,zg);
