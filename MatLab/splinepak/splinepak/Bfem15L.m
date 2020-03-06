% Bfem15L 6/27/14 
% SplinePAK: Copyright Larry Schumaker 2014
%   Solve a boundary-value problem using C1 quintic splines
% Version: L-shaped region

% Set up the PDE
kappa = @(x,y) 1;
f = @(x,y) 0;
u = @(x,y) urentrant(x,y);

% Read in a triangulation
[n,x,y,nt,TRI] = readtri;
%figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Calculate the transformation matrix
tic
[A,dof,bdof,dofb] = mds15f(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,vadj,eadj,tstart,tadj,bdy);
toc

d = 5;
% calculate the B-coeffs of the spline
[c,cb,M,t1,t2] = fem15(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,...
  area,bdy,A,dof,bdof,dofb,kappa,f,u);
fprintf('times to assemble and solve %g %g \n',t1,t2);

% Check the C1 smoothness of the spline
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid and plot
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(x); ymax = max(x);
[xg,yg,zg] = valspgridh(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

figure; surfl(xg,yg,zg'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,zg,u);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
