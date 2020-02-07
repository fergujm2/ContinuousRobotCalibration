% TPpenlsq
% SplinePAK: Copyright Larry Schumaker 2014
% Compute a penalized least squares fit using a tensor-product spline
%   using noisy data on a grid

% Define the sigmoidal test function
f = @(x,y) sigmoid(x,y);

% Set up the sample grid
a = 0; b = 1; aw = 0; bw = 1;
nd = 201; ndw = 201;
t = linspace(a,b,nd); tw = linspace(aw,bw,ndw);

% Evaluate f at the grid points
z = zeros(nd,ndw);
for i = 1:nd
 z(i,:) = f(t(i),tw);
end

% Add noise
eps = input('input eps ');
noise = readnoise(nd*ndw);
z = z +  eps*reshape(noise,nd,ndw);

% Input the degrees of the spline
d = input('input x-degree d '); dw = input('input y-degree dw ');

% Input the number of grid lines in each variable
m = input('input number of x-grid lines m = '); 
mw = input('input number of y-grid lines mw = ');

% Set up the extended knot sequences for  equally spaced knots
xe = [a*ones(1,d),linspace(a,b,m),b*ones(1,d)];
ye = [aw*ones(1,dw),linspace(aw,bw,mw),bw*ones(1,dw)];

% Compute the coef matrix for the penalized least squares fit 
lam = input('input lambda ');
c = penlsqtp(d,dw,xe,ye,lam,t,tw,z);

% Evaluate on a grid 
ng = 51; ngw = 51; nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,a,b,aw,bw);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,f);
fprintf('emax =%7.3e, RMS = %7.3e\n',norm(e,inf),erms(e));

% Plot the spline 
figure; surfl(xg,yg,g'); colormap(copper);
