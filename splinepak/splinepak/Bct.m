% Bct
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data with a C1 cubic Clough-Tocher spline

% Read in a triangulation
[no,xo,yo,nto,TRI] = readtri;

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Evaluate a test function at the data points
der = franke2d(xo,yo);
z = der(:,1); zx = der(:,2); zy = der(:,3);

% Compute the cross derivatives at midpoints of edges
x1 = xo(ie1o); x2 = xo(ie2o); y1 = yo(ie1o); y2 = yo(ie2o);
bd = franke2d((x1+x2)./2,(y1+y2)./2);
[dx,dy] =  ucross(x1,y1,x2,y2);
ze = dx.*bd(:,2) + dy.*bd(:,3);

% Compute the Clough-Tocher split and B-coefs
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  ct(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,z,zx,zy,ze);
toc
figure; triplot([v1,v2,v3],x,y);

% Check the C1 smoothness
d = 3; c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51; 
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors on the grid
e = errg(xg,yg,g,@franke2);
fprintf('Error:   emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
  xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
