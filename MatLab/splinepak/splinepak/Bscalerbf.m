% Bscalerbf
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a radial basis function
% Variant of Brbf that scales the problem

% Define the radial basic function and its derivative
rbf = @(eps,r) exp(-(eps*r).^2);  

% Input scattered points
[n,x,y] = readxy; 

% Input scaling parameters
lx = input('input x-scale factor lx = '); 
ly = input('input y-scale factor ly = ');
x = lx*x; y = ly*y; ctrs = [x,y];
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
ctrs = [x,y];

% Sample a scaled test function
f = @(x,y) franke2(x./lx,y./ly);
z = f(x,y);

% Input parameter for RBF basic function
eps = input('input eps '); ep2 = eps*eps;

tic
% Compute the RBF interpolant
[c,A] = rbfinterp(x,y,z,eps,rbf); cond(A)
toc

% Evaluate the RBF on a grid
ng = 51;  
tic
xg = linspace(xmin,xmax,ng);  yg = linspace(ymin,ymax,ng); 
[xe,ye] = meshgrid(xg,yg); epoints = [xe(:) ye(:)];
evalM = dismat(epoints,[x,y]);
EM = rbf(eps,evalM);
sval = EM * c;
toc

% Compute the values of the test function on the evaluation grid
xe1 = epoints(:,1); xe2 = epoints(:,2);
exact = f(xe1,xe2);

% Compute the max and RMS errors on the grid
err = exact - sval; ng2 = ng^2;
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n', norm(err,inf),erms(err))
g = reshape(sval,ng,ng);

% Plot the RBF
figure; surfl(xg,yg,g); colormap(copper); axis equal;

return

% Evaluate the derivative on the grid
ep2 = eps*eps;
dx = EM.*(-2*ep2*(repmat(xe1,1,n) - repmat(x',ng2,1)))*c;

% Plot the x-deriv
gx = reshape(dx,ng,ng);
figure; surfl(xg,yg,gx); colormap(copper);

% Calculate the error in x-derivative
der = franke2d(xe1,xe2); dex = der(:,2); errx = dx - dex;
fprintf('x-deriv: errmax =  %5.2e,  RMS = %5.2e\n', norm(errx,inf),erms(errx))
