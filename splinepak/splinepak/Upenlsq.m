% Upenlsq
% SplinePAK: Copyright Larry Schumaker 2014
% Find the penalized least squares fit to given (possibly noisy) data

% Define a function to generate the data
f = @(x) (x-.5).^2;

% Input the degree of the spline and number of knots
d = input('input the degree of the spline d = ');
k = input('input the number of knots k = '); 

% Find the dimension of the spline space
n = k+d+1;

% Set up the extended knot sequence
y(1:d) = zeros(1,d); y(n+2:n+d+1) = ones(1,d);
y(d+1:n+1) = linspace(0,1,n-d+1);

% Input the number of data points
nd = input('input the number of data points nd = ');

% Choose equally spaced sample points and evaluate f at these points
t = linspace(0,1,nd);
z = f(t);

% Add noise to the samples
eps = input('input eps ');
noise = readnoise(nd);
z = z + eps*noise';

% Input the parameter lambda controlling smoothing
lam = input('input lambda = ');

% Compute the spline coefficients
c = penlsq(d,n,y,t,z,lam);

% Find the max and RMS errors and plot
ng = 100;
xg = linspace(0,1,ng); fg = f(xg);
zg = sval2(d,y,c,xg); e = fg-zg;
fprintf('maxerr = %5.2e,  rms = %5.2e\n',norm(e,inf),erms(e));
figure; plot(xg,zg);
