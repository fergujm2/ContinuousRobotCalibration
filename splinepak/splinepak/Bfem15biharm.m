% Bfem15bharm 
% SplinePAK: Copyright Larry Schumaker 2014
% Solve biharmonic eqn with S^12_5 with homog bdy cndx

% Set up the PDE
% Example 9.32
u = @(x,y) exp(x+y); f = @(x,y) 4*u(x,y);
ub = u;
ux = u; uy = u;

% Example 9.33 on a disk
%u = @(x,y) x.^2.*(1-x).^2.*y.^2.*(1-y).^2;  % True solution
%f = @(x,y) 24*(x.^2.*(1-x).^2 + y.^2.*(1-y).^2) + ...
%   2*(2-12*x+12*x.^2).*(2-12*y+12*y.^2);

% Read a triangulation
[n,x,y,nt,TRI] = readtri;
triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
  vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Calculate the dof and transformation matrix
[A,dof,dofb] = mds15bh(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,vadj,eadj,tstart,tadj,bdy);
d = 5;

%a = size(A); cdof = rand(a(2),1); c = A*cdof; TEST mds15b

% Calculate the coefficients of the spline
[c,cb,M,t1,t2] = fem15bh(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,...
  area,bdy,A,dof,dofb,f,ub,ux,uy); 
fprintf('Condition number: %5.2e\n', condest(M));

% Check C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,u);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

return

% Render the boundary spline
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,cb,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);


% Plot the x-derivative
u = [1,0]; xmin = 0; xmax = 1; ymin = 0; ymax = 1;
[xg,yg,gx] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,gx');  colormap(copper);


% Plot the y-derivative
u = [0,1];
[xg,yg,gy] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,gy');  colormap(copper);
