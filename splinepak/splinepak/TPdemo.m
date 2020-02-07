% TPdemo
% SplinePAK: Copyright Larry Schumaker 2014
% Plot a tensor-product spline and its derivative and test integration

% Set the degrees and number of knots in each variable
d = 3; db = 3; n = 11; nb = 11;

% Compute the extended knot sequences
xe =  [0,0,0,linspace(0,1,n-2),1,1,1]; 
ye = [0,0,0,linspace(0,1,nb-2),1,1,1];

% Set the coefficients
c = zeros(n,nb); c(5,5) = 1;

% Input which derivative to plot
fprintf('Input mixed derivative orders \n');
nu = input('input nu '); mu = input('input mu ');

% Plot the spline
ng = 51; nbg = 51; 
a = 0; aw = 1; b = 0; bw = 1;
[xg,yg,g] = valtpgrid(d,db,xe,ye,c,nu,mu,ng,nbg,a,aw,b,bw);
figure; surfl(xg,yg,g'); colormap(copper);

return

% Compute the extended knot sequences and coefs for the antiderivative spline
[xi,yi,ci] = intcotp(d,db,xe,ye,c);

% Test integration on a rectangle [alpha,beta] x [alphaw,betaw]
alpha = input('input alpha ');
beta = input('input beta ');
alphaw = input('input alphaw ');
betaw = input('input betaw ');

int = intspltp(d,db,xi,yi,ci,alpha,beta,alphaw,betaw)

