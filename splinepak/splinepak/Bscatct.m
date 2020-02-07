% Bscatct 
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a C1 Clough-Tocher spline
%   using derivatives estimated by local polynomial least-squares

% Read in a triangulation
[no,xo,yo,nto,TRI] = readtri;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Sample a function at the vertices of the triangulation
%f = @(x,y) 3 + x + x.^3 + 2*y.^3; % TEST for reproduction of cubics
f = @(x,y) franke2(x,y);
zo = f(xo,yo);

% Estimate the needed derivatives
tic
[zx,zy,ze] = derestct(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,...
   adjstarto,vadjo);

% Compute the coefs of the spline
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  ct(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,...
      trilo,triro,zo,zx,zy,ze);
toc

% Check the C1 smoothness
d = 3; c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('Error:  emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
