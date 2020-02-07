% TPpsdat
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a two-stage method.  The first stage
%   computes a C1 Powell-Sabin spline with estimated derivatives.
%  It is then converted to a tensor-product spline using quasi-interpolation

% Read scattered data points from file -- bk.dat for the Black Forest
[no,xo,yo,zo] = readxyz;
xmin =  min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Compute the Delaunay triangulation
TRI = delaunay(xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Compute the scatps interpolant
tic
de = 3; m = 20;
[zxo,zyo] =  derestlsq(xo,yo,zo,adjstarto,vadjo,de,m);
toc

tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,zo,zxo,zyo);
toc

% Choose the number of knot lines to use
nx = input(' input number of x-grid lines ');
ny = input(' input number of y-grid lines ');
d = 2;

% Compute the coefficients of the tensor-product spline
tic
[xe,ye,ctp] = sp2tp(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,xmin,xmax,ymin,ymax,nx,ny);
toc

% Compute the error at the interpolation points
e = zeros(1,no);
mx = nx+2; my = ny+2;
for i = 1:no;
 xi = xo(i); yi = yo(i);
  l = findinterval (mx,ye,xi);
  lw = findinterval (my,ye,yi);
  e(i) = valtp(3,3,xe,ye,ctp,l,lw,xi,yi) -zo(i);
end
fprintf('Discrepency: max =%5.2e, rms = %5.2e\n',norm(e,inf),erms(e));

% Evalute the tensor-product spline on a grid
x1 = input('input x1 ');  x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
ng = 51; ngw = 51;  nu = 0; mu = 0;
[xg,yg,g] = valtpgrid(3,3,xe,ye,ctp,nu,mu,ng,ngw,x1,y2,y1,y2);

% Plot the tensor-product spline
figure; surfl(xg,yg,g'); colormap(copper);

