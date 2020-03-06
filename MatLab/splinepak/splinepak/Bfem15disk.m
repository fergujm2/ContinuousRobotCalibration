% Bfem15disk
% SplinePAK: Copyright Larry Schumaker 2014
% Solve 2nd Order BVP with Dirichlet boundary conditions
%   Using S^{1,2}_5_ spline space on a disk


% Set up the PDE and boundary conditions
% Example 9.27
kappa = @(x,y) 1;
f = @(x,y) 20*x.^3.*y.^2 +6*(x.*y.^4 - x.*y.^2) + 12*y.^2.*x.^3 ...
    + 2*(x.^5 - x.^3);
u = @(x,y) (1-x.^2 - y.^2).*(x.^3.*y.^2);   % True solution
g = u;  %BC

% Input a triangulation
[n,x,y,nt,TRI] = readtri;
triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
  vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Calculate the dof and transformation matrix
[A,dof,bdof,dofb] = mds15f(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,vadj,eadj,tstart,tadj,bdy);
d = 5;

% Calculate the B-coeffs of Rayleigh-Ritz spline
[c,cb,M,t1,t2] = fem15(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,...
  area,bdy,A,dof,bdof,dofb,kappa,f,g);
fprintf('times to assemble and solve %g %g \n',t1,t2);
fprintf('size M %g,  condition number %5.2e \n',length(M),condest(M));

% Check the C1 smoothness of the spline
d = 5;
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

%  Evaluate the spline on an enclosing rectangle
ng = 71; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Calculate the error on the grid
e = errg(xg,yg,g,u);
fprintf('Maximum error: %5.2e,  RMS = %5.2e\n', norm(e,inf),erms(e));

% Replace the NaN's in g by zeros for a smoother looking plot
b = find(isnan(g));  g(b) = zeros(length(b),1);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);
