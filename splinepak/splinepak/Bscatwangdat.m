% Bscatwangdat
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered dat with a C2 quintic spline
%    based on the Wang macro-element
% Version of Bscatwang that reads data and computes the Delaunay  triangulation


% Read in data points and values
[no,xo,yo,zo] = readxyz;

% Compute the Delaunay triangulation
tic
TRI = delaunay(xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);
toc
%triplot(TRI,xo,yo);


% Estimate the needed derivatives
tic
[der,ze] = derestwang(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,adjstarto,vadjo);
toc

tic
% Compute the Wang triangulation and the coef vector c
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] ...
   = wang(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,der,ze);
toc
d = 5;

% Check the 1st and 2nd order smoothness
%cksmooth(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51;
a = input('input a '); b = input('input b ');
aw = input('input aw '); bw = input('input bw ');
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,a,b,aw,bw);
toc

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;
