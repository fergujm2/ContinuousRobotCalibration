%  Ulsqopt 
% SplinePAK: Copyright Larry Schumaker 2014
% Compute a least squares spline with an optimal choice of knots
%   subject to a minimal spacing between the knots (may be zero)

% Define the function to be fit
f = @(x) (0<= x & x < .5).*(1-2*x) + (.5<= x & x <= 1).*(x-.5);

% Input the degree of the spline and number of interior knots
d = input('input the degreee of the spline d = ');
k = input('input the number of knots k = ');

% Find the dimension of the spline space
n = k+d+1;

% Input the number of data points
nd = input('input the number of data points nd = ');
t = linspace(0,1,nd);
z = zeros(1,nd);

% Choose equally spaced interior knots as the initial guess
h = (t(nd)-t(1))/(k+1);
x = linspace(t(1)+h,t(nd)-h,k);
z = f(t);

% Set the extended knot sequence and compute lsq spline fit
y = [t(1)*ones(1,d+1),x,t(nd)*ones(1,d+1)];
c = lsqspl(d,n,y,t,z);

%Compute the errors and plot
ng = 100;  xg = linspace(0,1,ng); fg = f(xg);
zg = sval2(d,y,c,xg); e= fg-zg;
fprintf('emax = %5.2e,  rms error = %5.2e\n',norm(e,inf),erms(e));
figure; plot(xg,zg);

% Input the minimal spacing allowed between knots
minspace = input('input minspace = ');

% Compute the optimal locations for the interior knots
x = lsqopt(d,k,t,z,x,minspace);

% Set the extended knot sequence and compute lsq spline fit
y = [t(1)*ones(1,d+1),x,t(nd)*ones(1,d+1)];
c = lsqspl(d,n,y,t,z);

%Compute the errors and plot
ng = 100;  xg = linspace(0,1,ng); fg = f(xg);
zg = sval2(d,y,c,xg); e = fg-zg;
fprintf('emax = %5.2e,  rms error = %5.2e\n',norm(e,inf),erms(e));
figure; plot(xg,zg);

