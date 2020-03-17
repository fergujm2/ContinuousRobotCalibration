% Bpsrand
% SPAK: Copyright Larry Schumaker 2012
% Interpolate Hermite data with a C^1 quadratic Powell-Sabin spline
%  on a Delaunay triangulation of n data points to be read from randpts.dat

% Read in the triangulation
no = input('input number of interpolation points no =  ');
[xo,yo] = randxy(no);

TRI = delaunay(xo,yo);
figure; triplot(TRI,xo,yo); axis equal; axis off;

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Sample a function to get the nodal data
der = franke2d(xo,yo);
z = der(:,1); zx = der(:,2); zy = der(:,3);

% Compute the Powell-Sabin triangulation and the coef vector c
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,z,zx,zy);
d = 2;

% Evaluate the spline on a grid
ng = 51;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('Error:   emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

return

% Plot the x-derivative
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));

