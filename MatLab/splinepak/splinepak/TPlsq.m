% TPlsq
% SplinePAK: Copyright Larry Schumaker 2014
% Compute a least squares fit to gridded data using a tensor-product spline

% Set up the sample grid
a = 0; b = 1; aw = 0; bw = 1;
nd = 201; ndw = 201;
t = linspace(a,b,nd); tw = linspace(aw,bw,ndw);

% Evaluate franke2 at the grid points to get the data
z = zeros(nd,ndw);
f = @(x,y) franke2(x,y);
for i = 1:nd
 z(i,:) = f(t(i),tw);
end

% Input the degrees of the spline
d = input('input degree in x-variable d = '); 
dw = input('input degree in y-variable dw = ');

% Input the number of grid lines in each variable
m = input('input number of x-grid lines m = '); 
mw = input('input number of y-grid lines mw = ');

% Set up the extended knot sequences for  equally spaced knots
xe = [a*ones(1,d),linspace(a,b,m),b*ones(1,d)];
ye = [aw*ones(1,dw),linspace(aw,bw,mw),bw*ones(1,dw)];

% Compute the coef matrix
c = lsqtp(d,dw,xe,ye,t,tw,z);

% Evaluate on a grid 
ng = 51; ngw = 51; nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,a,b,aw,bw);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline 
figure; surfl(xg,yg,g'); colormap(copper);
