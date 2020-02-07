% Bscalemenps
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a minimal energy
%   C1 Powell-Sabin spline using a nodal determining set
% Variant of Bmenps to explore scaling

% Read in a triangulation
[no,xo,yo,nto,TRI] = readtri;
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRI);
lx = input('input lx '); ly = input('input ly ');
xo = lx*xo; yo = ly*yo;

% Sample a scaled function
f = @(x,y) franke2(x./lx,y./ly);
zo = f(xo,yo);

% Compute the Powell-Sabin split, degrees of freedom, and
%    transformation matrix
to = cputime;
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
tm = cputime - to;

% Compute the minimal energy C^1 quadratic Powell-Sabin interpolant
[c,M,t1,t2] = menps(v1o,v2o,v3o,x,y,zo, ...
   v1,v2,v3,e1,e2,e3,ie1,A);
fprintf('Time: nmds, assemble and solve %g %g %g \n',tm,t1,t2);

% Check the C1 smoothness
d = 2;
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the the spline on a grid
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('Error:  emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Compute the mesh size
mesh = max(sqrt((x(ie2)-x(ie1)).^2 + (y(ie2)-y(ie1)).^2));
fprintf('Mesh size = %5.2e\n', mesh);
