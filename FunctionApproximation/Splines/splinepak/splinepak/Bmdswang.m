% Bmdswang
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Construct a minimal determing set dof for the C^2
% Wang macro-element space and find the transformation matrix A
% Plots individual basis functions

% Read in the triangulation
[no,xo,yo,nto,TRI] = readtri;
figure; triplot(TRI,xo,yo);

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Find degrees of freedom and the transformation matrix
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,tadj,tstart, ...
   dof,A]  = mdswang(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o, ...
     ie1o,ie2o,trilo,triro,tstarto,tadjo);
toc

figure; triplot([v1,v2,v3],x,y);

cdof = zeros(length(dof),1);
id = input ('input number of basis function to plot ');
cdof(id) = 1;
c = A*cdof;

% Check the 1st and 2nd order smoothness
d = 5;
cksmooth(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Render the spline
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);
