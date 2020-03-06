% Bscatlocc
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates scattered data using a C1 cubic Clough-Tocher spline
% This is a two-stage local method, not using estimated derivatives 

% Read in data points and an associated triangulation
[no,xo,yo,nto,TRI] = readtri;
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Sample a test function at the data points
zo = franke2(xo,yo);

% Compute the coefs of the interpolating spline
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  scatlocct(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,...
    adjstarto,vadjo);
toc
d = 3;

% Check for C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51; 
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors on the grid
e = errg(xg,yg,g,@franke2);
fprintf('Error:   emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the x-derivative -- 
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);


% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
