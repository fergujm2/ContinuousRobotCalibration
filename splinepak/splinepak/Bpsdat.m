% Bpsdat
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate scattered data using a Powell-Sabin spline
%   with estimated derivatives
%   Version of Bps that inputs data points and creates a Delaunay triangulation

% Read in data  --- Use bk.dat for the Black Forest example
[no,xo,yo,zo] = readxyz;
xmin =  min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Compute Delaunay triangulation
tic
 TRI = delaunay(xo,yo);
toc

% Calculate the triangulation lists
tic
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);
toc

% Estimate the derivatives by local LSQ
tic
de = 3; m = 20; [zxo,zyo] =  derestlsqk(xo,yo,zo,de,m);
toc

% compute the coefs of the Powell-Sabin interpolant
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,zo,zxo,zyo);
toc

d = 2
% Check the C^1 smoothness
%c1ck(2,n,ne,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Render on a subrectangle
fprintf('input a rectangle for plotting \n');
x1 = input('input x1 ');  x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');

ng = 51;
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);
toc
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

