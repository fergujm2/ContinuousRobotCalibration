% Bpsf0
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data with a C^1 quadratic Powell-Sabin spline
% Version: Uses a test function that is only C0

% Define the test function
f = @(x,y) (y >= 1-2*x).*(y+2*x-1);
fx = @(x,y) 2*(y >= 1-2*x); fy = @(x,y) (y>=1-2*x);;

% Read in the triangulation
[no,xo,yo,nto,TRI] = readtri;       %figure; triplot(TRI,xo,yo);

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Sample the function to get the nodal data
z = f(xo,yo); zx = fx(xo,yo); zy = fy(xo,yo);

% Compute the Powell-Sabin triangulation and the coef vector c
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,z,zx,zy);
toc

% Check the C^1 smoothness
d = 2; c1ck(2,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51; 
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
