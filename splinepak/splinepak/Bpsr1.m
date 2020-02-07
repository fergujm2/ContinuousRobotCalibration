% Bpsr1
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data with a C1 quadratic Powell Sabin spline
% Version: Uses a type-1 triangulation of a rectangle

%  Set up a type-1 triangulation
%xmin = 0; xmax = 100; ymin = 0; ymax = 10;
xmin = 0; xmax = 10; ymin = 0; ymax = 1;
nx = input('input nx '); ny = input('input ny ');
[xo,yo,TRI] = type1(nx,ny,xmin,xmax,ymin,ymax);
figure; triplot(TRI,xo,yo); axis equal; axis off;
lx = xmax-xmin; ly = ymax-ymin;

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Sample a function to get the nodal data
f = @(x,y) franke2(x./lx,y./ly);
der = franke2d(xo/lx,yo/ly);
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
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

mesh = max(sqrt((x(ie2)-x(ie1)).^2 + (y(ie2)-y(ie1)).^2));
fprintf('mesh size = %5.2e\n', mesh);

return

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
[emax,erms] = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',emax,erms);

