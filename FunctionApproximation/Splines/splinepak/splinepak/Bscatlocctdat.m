% Bscatlocctdat
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates scattered data using a C1 cubic Clough-Tocher spline
% This is a two-stage local method, not using estimated derivatives 
% Version of Bscatlocct, but reads data and creates a Delaunay triangulation


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

% Compute the coefs of the interpolating spline
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  scatlocct(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,...
   trilo,triro,adjstarto,vadjo);
toc
d = 3;

% Evaluate the spline on a rectangular grid
x1 = input('input x1 '); x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
ng = 51; 
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);
toc

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

return

% Check interpolation
i = input('input point number ');
[zo(i),valsp(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,xo(i),yo(i))]
