% Brbf
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a radial basis function
% Uses the Gaussian basic function with parameter eps

% Define the radial basic function and its derivative
rbf = @(eps,r) exp(-(eps*r).^2);  

% Input scattered points
[n,x,y] = readxy; ctrs = [x,y];
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);

% Sample Franke's function at the data sites
z = franke2(x,y);

% Input parameter for RBF basic function
eps = input('input eps '); ep2 = eps*eps;

tic
% Compute the RBF interpolant
[c,A] = rbfinterp(x,y,z,eps,rbf); cond(A)
toc

% Evaluate the RBF interpolant on a grid
ng = 51;  
tic
xg = linspace(xmin,xmax,ng); yg = linspace(ymin,ymax,ng);
[xe,ye] = meshgrid(xg); epoints = [xe(:) ye(:)];
evalM = dismat(epoints,ctrs);
EM = rbf(eps,evalM);
sval = EM * c;
toc

% Compute the values of Franke's function on the evaluation grid
xe1 = epoints(:,1); xe2 = epoints(:,2);
exact = franke2(xe1,xe2);

% Compute the errors
err = exact - sval; ng2 = ng^2;
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n', norm(err,inf),erms(err))
g = reshape(sval,ng,ng);
figure; surfl(xg,yg,g); colormap(copper);

% Evaluate the derivative on the grid
ep2 = eps*eps;
dx = EM.*(-2*ep2*(repmat(xe1,1,n) - repmat(x',ng2,1)))*c;

% Plot the x-deriv
gx = reshape(dx,ng,ng);
figure; surfl(xg,yg,gx); colormap(copper);

% Compute the error in the x-derivative
der = franke2d(xe1,xe2); dex = der(:,2); errx = dx - dex;
fprintf('Dx: Maximum error: %5.2e,   RMS = %5.2e\n', norm(errx,inf),erms(errx))
