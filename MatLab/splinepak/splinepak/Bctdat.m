% Bctdat 
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate scattered dat with a Clough-Tocher spline
%  using the Delaunay triangulation and estimated derivatives

% Read data from bk.dat
[no,xo,yo,zo] = readxyz;
xmin =  min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Compute Delaunay triangulation
tic
 TRI = delaunay(xo,yo);
toc

% Compute the triangulation lists
tic
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);
toc

% Estimate the needed derivatives for the CT interpolant
tic
[zx,zy,ze] = derestct(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,adjstarto,vadjo);
toc

% Compute the C1 Clough-Tocher interpolant
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  ct(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,...
      trilo,triro,zo,zx,zy,ze);
toc

d = 3; 
% Check the C^1 smoothness
%c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Render the spline
tic
ng = 101;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
toc
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

% Render on a subrectangle
fprintf('input a rectangle ');
x1 = input('input x1 ');  x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
ng = 81;

tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);
toc
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

