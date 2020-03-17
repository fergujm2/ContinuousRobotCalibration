% Bmenpshole 
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a minimal energy
%   C^1 Powell-Sabin spline using a nodal determining set
% Version: Renders splines on domains with holes properly

% Read in a triangulation
[no,xo,yo,nto,TRI] = readtri;
triplot(TRI,xo,yo);
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Calculate the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRI);

% Sample a function at the vertices of the triangulation
zo = franke2(xo,yo);

% Compute the Powell-Sabin split, degrees of freedom, and
%    transformation matrix
to = cputime;
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
tm = cputime - to;
triplot([v1,v2,v3],x,y);

% Compute the minimal energy C^1 quadratic Powell-Sabin interpolant
[c,M,t1,t2] = menps(v1o,v2o,v3o,x,y,zo, ...
   v1,v2,v3,e1,e2,e3,ie1,A);
fprintf('Time: nmds, assemble and solve %g %g %g \n',tm,t1,t2);

% Check the C1 smoothness
d = 2;
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evluate the spline at grid points in the domain
ng = 51;
[xg,yg,g] = valspgridh(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('Error:  emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
