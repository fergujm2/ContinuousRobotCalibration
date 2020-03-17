% Bpsf1
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data with a C1 quadratic Powell-Sabin spline
% Version: Uses a test function that is only C1

f = @(x,y) (y >= 1-2*x).*(y+2*x-1).^2;
fx = @(x,y) 4*(y >= 1-2*x).*(y+2*x-1);
fy = @(x,y) 2*(y >= 1-2*x).*(y+2*x-1);

% Read in the triangulation
[no,xo,yo,nto,TRI] = readtri;        %figure; triplot(TRI,xo,yo);

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Sample the function to get the nodal data
z = f(xo,yo); zx = fx(xo,yo); zy = fy(xo,yo);

% Compute the Powell-Sabin triangulation and the coef vector c
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,z,zx,zy);

% Check the C1 smoothness
d = 2; c1ck(2,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51; 
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
toc
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors on the grid
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

