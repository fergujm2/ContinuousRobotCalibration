% Bscatpsdat
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a C1 Powell-Sabin spline
%   using derivatives estimated from the Lagrange data 
% Version of Bscatps that reads data and uses Delaunay triangulation

% Read in data points and values
[no,xo,yo,zo] = readxyz;

% Compute the Delaunay triangulation
tic
TRI = delaunay(xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);
toc
%triplot(TRI,xo,yo);

% Estimate the gradients at the vertices
tic
m = 20; de = 3; 
[zx,zy] =  derestlsq(xo,yo,zo,adjstarto,vadjo,de,m); 
toc

% Compute the  C1 quadratic Powell-Sabin interpolant
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,zo,zx,zy);
toc

% Evaluate the spline on a rectangle
x1 = input('input x1 '); x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
ng = 81; d = 2;
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);
toc

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;
