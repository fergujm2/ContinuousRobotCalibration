% Bscat15dat
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a C1 quintic Argyris spline
% Uses derivatives estimated by local polynomial least-squares
% Version of Bscat15 that reads data and computes the Delaunay triangulation

% Read in data points and values
[n,x,y,z] = readxyz;

% Compute the Delaunay triangulation
tic
TRI = delaunay(x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
toc
triplot(TRI,x,y);

% Estimate the needed derivatives
tic
[der,ze] = derest15(x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,adjstart,vadj);
toc

% Compute the coefs of the interpolating spline
tic
c =  arg15(x,y,v1,v2,v3,e1,e2,e3,ie1,der,ze);
toc

% Check the C1 smoothness
d = 5;
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51;
x1 = input('input x1 '); x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
[xg,yg,g] =  valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;
