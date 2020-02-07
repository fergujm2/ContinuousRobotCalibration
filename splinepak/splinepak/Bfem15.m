% Bfem15 
% SplinePAK: Copyright Larry Schumaker 2014
% Solve a 2nd Order BVP with mixed boundary conditions
%   using the Argyris macro-element spline space S^12_5

% Example 9.24
kappa = @(x,y) exp(x+y);
u = @(x,y) -sin(4*x) - sin(4*y);
f = @(x,y) exp(x+y).*(-16*sin(4*x) - 16*sin(4*y) + 4*cos(4*x) + 4*cos(4*y));
g = u;

% Example 9.25
%kappa = @(x,y) 1;
%u = @(x,y) sin(x.^2+y.^2) + .1*sin(25*(x.^2+y.^2));
%f = @(x,y) -4.*cos(x.^2+y.^2) + 4*(x.^2+y.^2).*sin(x.^2+y.^2) ... 
%  -10*cos(25*(x.^2+y.^2)) + 250*(x.^2+y.^2).*sin(25*(x.^2+y.^2));
%g = u;

% Input a triangulation
[n,x,y,nt,TRI] = readtri;       %triplot(TRI,x,y);
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
  area,bdy,A,dof,bdof,dofb,kappa,f,g);
fprintf('times to assemble and solve %g %g \n',t1,t2);


% Check the C^1 smoothness of the spline
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a rectangular grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,u);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline s_b interpolating the boundary data
[xg,yg,gb] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,cb,...
  ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,gb'); colormap(copper);

% Compute the condition number of the stiffness matrix
fprintf('size M %g,  condition number %5.2e \n',length(M),condest(M));
