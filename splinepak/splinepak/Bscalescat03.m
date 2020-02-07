% Bscalelocct
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates scattered data using a C1 cubic Clough-Tocher spline
% This is a two-stage local method, not using estimated derivatives 
% Version of Bscatlocct to explore scale invariance

% Read in data points and an associated triangulation
[n,x,y,nt,TRI] = readtri;
% Choose the scale factors and scale the domain
lx = input('input lx '); ly = input('input ly ');
x = lx*x; y = ly*y;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
figure; triplot(TRI,x,y);

[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);


% Sample a scaled function
f = @(x,y) franke2(x./lx,y./ly);
z = f(x,y);

% Compute the coefs of the interpolating spline
tic
c = scat03(x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,adjstart,vadj);
toc
d = 3;

% Check for C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51; 
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

% Compute the max and RMS errors on the grid
e = errg(xg,yg,g,f);
fprintf('Error: emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

