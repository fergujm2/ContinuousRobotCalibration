%  B01r1 
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate data on type-1 triangulation of a rectangle
%   with a C^0 linear spline. Uses a scaled function

% Set up the rectangle
xmin = 0; xmax = 100; ymin = 0; ymax = 10;
lx = xmax-xmin; ly = ymax-ymin;

% Choose numbers of grid lines
nx = input('input number of grid lines nx '); 
ny = input('input number of grid lines ny ');

% Find a corresponding type-1 triangulation
[x,y,TRI] = type1(nx,ny,xmin,xmax,ymin,ymax);
figure; triplot(TRI,x,y); axis equal;

% Sample a test function to get the coefs of the interpolating spline
f = @(x,y) franke2(x./lx,y./ly);
c = f(x,y);

% Compute the triangulation lists (needed for rendering)
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Evaluate the spline on a 51 x 51 grid on the rectangle
ng = 51; d = 1;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

%  Compute the mesh size
mesh = max(sqrt((x(ie2)-x(ie1)).^2 + (y(ie2)-y(ie1)).^2));
fprintf('mesh size = %5.2e\n',mesh);
