% Bscatctdat
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a C1 Clough-Tocher spline
%   using derivatives estimated from the Lagrange data 
% Version of Bscatct that reads data and computes a Delaunay triangulation

% Read in data points and values
[no,xo,yo,zo] = readxyz;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Compute the Delaunay triangulation
tic
TRI = delaunay(xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);
toc
%triplot(TRI,xo,yo);

% Estimate the needed derivatives
tic
[zx,zy,ze] = derestct(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,...
   adjstarto,vadjo);
toc

% Compute the coefs of the spline
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  ct(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,...
      trilo,triro,zo,zx,zy,ze);
toc

% Check the C1 smoothness
d = 3; c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51;
x1 = input('input x1 '); x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);
toc

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;
