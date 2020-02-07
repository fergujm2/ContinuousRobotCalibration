% Bnmdsps
% SplinePAK: Copyright Larry Schumaker 2014
% Finds a nodal minimal determining set for the C1 Powell-Sabin space
% Also plots individual M-basis functions

% Read in a triangulation
[no,xo,yo,nto,TRI] = readtri;     %figure; triplot(TRI,x,y);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Refine the triangulation and compute the transition matrix A
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
toc

% Plot the refined triangulation
TRI = [v1,v2,v3]; figure; triplot(TRI,x,y);

% Set one degree of freedom to one, and all others to zero
sa = size(A); ndof = sa(2); cdof = zeros(ndof,1);
m = input('choose dof to set to one m = ');
cdof(m) = 1;

% Compute all B-coefs of the basis function
c = A*cdof; d = 2;

% Check for C1 continuity
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the basis function on the smallest enclosing rectangle
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[gx,gy,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the basis function
figure; surfl(gx,gy,g'); colormap(copper);
