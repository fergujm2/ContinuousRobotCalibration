% Bfem15mix 
% SplinePAK: Copyright Larry Schumaker 2014
% Solve 2nd Order BVP with mixed boundary conditions
%   Using S^0_d spline space

% Example 9.28
kappa = @(x,y) exp(x+y);
u = @(x,y) -sin(4*x) - sin(4*y);
f = @(x,y) exp(x+y).*(-16*sin(4*x) - 16*sin(4*y) + 4*cos(4*x) + 4*cos(4*y));
g = u;
ux = @(x,y) -4*cos(4*x); uy = @(x,y) -4*cos(4*y);
h = @(x,y) (ux(x,y).*(-(x==0) + (x==1)) + uy(x,y).*(-(y==0) + ...
     (y==1))).*kappa(x,y);

% input a triangulation
[n,x,y,nt,TRI] = readtri;
triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
  vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Calculate the dof and transformation matrix
[A,dof,bdof,dofb] = mds15f(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,vadj,eadj,tstart,tadj,bdy);
d = 5;

% Mark boundary edges
nb = 0; eb = zeros(1,3);
for i = 1:ne
  if tril(i) == 0 | trir(i) == 0
    nb = nb+1; eb(nb) = i;
  end
end

% Mark boundary edges with Neumann BC
ebn = zeros(1,nb);
for i = 1:nb
  j = eb(i);
  if y(ie1(j)) == 0 & y(ie2(j)) == 0  % bottom edge
     ebn(i) = 1;
  end
  if x(ie1(j)) == 1 & x(ie2(j)) == 1  % right edge
     ebn(i) = 1;
  end
end

% Calculate the B-coeffs of the RR spline
[c,M,t1,t2] = fem15mix(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,...
  area,bdy,kappa,A,dof,bdof,dofb,eb,ebn,f,g,h);
fprintf('times to assemble and solve %g %g \n',t1,t2);

% compute condition number of the stiffness matrix
fprintf('size M %g,  condition number %5.2e \n',length(M),condest(M));

% Check the C1 smoothness of the spline
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,u);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
