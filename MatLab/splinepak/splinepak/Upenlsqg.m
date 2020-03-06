% Upenlsqg
% SplinePAK: Copyright Larry Schumaker 2014
% Test penalized least squares with a sequence of lambdas
%   and plot the corresponding RMS errors

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

% create lambda values and compute associated RMS errors
np = 31;  lam = linspace(0,.002,np); e2 = zeros(1,np);
ng = 100; xg = linspace(0,1,ng); fg = f(xg);
for i = 1:np
  c = penlsq(d,n,y,t,z,lam(i));
  zg = sval2(d,y,c,xg); e2(i) = erms(fg-zg);
  fprintf('lam = %10.3e, error = %10.3e \n',lam(i),e2(i));
end

% Interpolate the  RMS errors with a not-a-knot spline and plot
[yp,cp] = notaknot(3,lam,e2);
zp = sval2(3,yp,cp,lam);
figure; plot(lam,zp);

