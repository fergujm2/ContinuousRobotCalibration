% Bscaleps
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data using the C1 Powell-Sabin space
% Variant of Bps to explore scaling

% Read in a triangulation and scale it
[no,xo,yo,nto,TRI] = readtri;
lx = input('input lx '); ly = input('input ly ');
xo = lx*xo; yo = ly*yo;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);
figure; triplot(TRI,xo,yo); 

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Sample a scaled function to get the nodal data
f = @(x,y) franke2(x./lx,y./ly);
der = franke2d(xo./lx,yo./ly);
z = der(:,1); zx = der(:,2)/lx; zy = der(:,3)/ly;

tic
% Compute the Powell-Sabin triangulation and the coef vector c
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,z,zx,zy);
toc 

% Check the C1 smoothness
d = 2; c1ck(2,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51; 
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% compute the mesh size
mesh = max(sqrt((x(ie2)-x(ie1)).^2 + (y(ie2)-y(ie1)).^2));
fprintf('mesh size = %5.2e\n', mesh);

return

% Plot the x-derivative on the grid
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));

