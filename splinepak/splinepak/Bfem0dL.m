% Bfem0dL 
% SplinePAK: Copyright Larry Schumaker 2014
% Solve a 2nd order PDE on an L-shaped domain with Dirichlet boundary cndx
%   using the spline space S0d

% Set up the PDE
kappa = @(x,y) 1;
f = @(x,y) 0;
u = @(x,y) urentrant(x,y);

% Read in a triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Choose a degree
d = input('input the degree of the spline d =  ');

% Compute the coefs of the Rayleigh-Ritz spline
if d == 1 
  [c,M,t1,t2] = fem01(x,y,v1,v2,v3,e1,e2,e3,ie1,area,bdy,kappa,f,u);
else
  [c,M,t1,t2] = fem0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,...
     tril,trir,area,bdy,kappa,f,u);  
end
fprintf('times to assemble and solve %g %g \n',t1,t2);

% Evaluate the spline on a rectangular grid 
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(x); ymax = max(x);
[xg,yg,zg] = valspgridh(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

figure; surfl(xg,yg,zg'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,zg,u);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
