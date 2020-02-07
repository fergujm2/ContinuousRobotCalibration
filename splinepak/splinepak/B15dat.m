% B15dat 8/8/14
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate scattered data with a C^1 quintic Argyris spline
%  Uses the Delaunay triangulation of the data
%  Uses first and 2nd derivatives estimated from the scattered data values

% Read in data 
[n,x,y,z] = readxyz;
xmin =  min(x); xmax = max(x); ymin = min(y); ymax = max(y);

% Compute Delaunay triangulation
tic
 TRI = delaunay(x,y);
toc

% Calculate the triangulation lists
tic% SplinePAK: Copyright Larry Schumaker 2014% SplinePAK: Copyright Larry Schumaker 2014
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
toc

% Estimate derivative info at vertices of argyris
tic
[der,ze] = derest15(x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,adjstart,vadj);
toc

tic
c =  arg15(x,y,v1,v2,v3,e1,e2,e3,ie1,der,ze);
toc

d = 5;
% Check the C^1 smoothness
%c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
tic
ng = 101;
[xg,yg,g] =  valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
toc

% Render the spline on the rectangle
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

% Render on a subrectangle
x1 = input('input x1 ');  x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
ng = 81;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

