%  Bscale01
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate data at the vertices of a triangulation
%    with a C0 linear spline
% Viant of B01 to study scale invariance

% Read in and plot the triangulation
[n,x,y,nt,TRI] = readtri;
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
figure; triplot(TRI,x,y); axis equal; 

% Choose the scale factors and scale the domain
lx = input('input lx '); ly = input('input ly ');
x = lx*x; y = ly*y;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
figure; triplot(TRI,x,y);

% Sample a scaled function
f = @(x,y) franke2(x./lx,y./ly);
c = f(x,y);

% Evaluate the spline on a grid
ng = 51; d = 1;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the psline
figure; surfl(xg,yg,g'); colormap(copper); axis equal;

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Compute the mesh size
meshsize = max(sqrt((x(ie2)-x(ie1)).^2 + (y(ie2)-y(ie1)).^2));
fprintf('mesh size = %5.2e\n',meshsize);
