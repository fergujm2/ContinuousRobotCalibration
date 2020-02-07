% Bscatwang
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered dat with a C2 quintic spline
%    based on the Wang macro-element

% Read in the triangulation
[no,xo,yo,nto,TRI] = readtri;
figure; triplot(TRI,xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Sample a test function
%f = @(x,y) 1 + 2*x + (x+2*y).^3;  Test reproduction of cubics
f = @(x,y) franke2(x,y);
zo = f(xo,yo);

% Estimate the needed derivatives
tic
[der,ze] = derestwang(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,adjstarto,vadjo);

% Compute the Wang triangulation and the coef vector c
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] ...
   = wang(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,der,ze);
toc

% Check the 1st and 2nd order smoothness
d = 5;
cksmooth(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('Error: emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
  xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
