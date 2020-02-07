% Bfem15bihdisk 12/8/13
% Solve biharmonic eqn with S^12_5 with homog bdy cndx on a DISK

% Example 9.33
u = @(x,y) (1-x.^2 - y.^2).*(x.^3.*y.^2); 
f = @(x,y) -120*x.*y.^2 - 24*x.^3 + 2*(12*x - 40*x.^3 -72*x.*y.^2);
ux = @(x,y) 3*x.^2.*y.^2 - 5*x.^4.*y.^2 - 3*x.^2.*y.^4;
uy = @(x,y) 2*x.^3.*y -2*y.*x.^5 - 4*x.^3.*y.^3;
ub = u;

% Read a triangulation
[n,x,y,nt,TRI] = readtri;
triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
  vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Calculate the dof and transformation matrix
[A,dof,dofb] = mds15bh(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,vadj,eadj,tstart,tadj,bdy);
d = 5;

% Solve the coefs of the spline
[c,cb,M,t1,t2] = fem15bh(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,...
  area,bdy,A,dof,dofb,f,ub,ux,uy); 
fprintf('times to assemble and solve %g %g \n',t1,t2);
fprintf('size M %g,  condition number %5.2e \n',length(M),condest(M));

% Check C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

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

return

% Plot the x-derivative
u = [1,0]; 
[xg,yg,gx] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,gx');  colormap(copper);

% Plot the y-derivative
u = [0,1];
[xg,yg,gy] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,gy');  colormap(copper);

