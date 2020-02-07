% Bwang
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data with the C2 quintic Wang macro-element spline

% Read in the triangulation
[no,xo,yo,nto,TRI] = readtri;
%figure; triplot(TRI,xo,yo);

% Compute the triangulation lists
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

ze = zeros(neo,3);
x1 = xo(ie1o); x2 = xo(ie2o); y1 = yo(ie1o); y2 = yo(ie2o);
bd1 = zeros(neo,6);   bd2 = ones(neo,3);   bd3 = zeros(neo,6);

% Sample Franke's function to get the nodal data at the vertices
der = franke2d(xo,yo);

% Compute the cross derivative information
bd1 = franke2d((2*x1+x2)./3,(2*y1+y2)./3);
bd2 = franke2d((x1+x2)./2,(y1+y2)./2);
bd3 = franke2d((x1+2*x2)./3,(y1+2*y2)./3);
[dx,dy] =  ucross(x1,y1,x2,y2);
ze(:,1) = dx.^2.*bd1(:,4) + 2*dx.*dy.*bd1(:,5) + dy.^2.*bd1(:,6);
ze(:,2) = dx.*bd2(:,2) + dy.*bd2(:,3);
ze(:,3) = dx.^2.*bd3(:,4) + 2*dx.*dy.*bd3(:,5) + dy.^2.*bd3(:,6);

% Compute the coefs of the spline
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] ...
   = wang(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,der,ze);
toc

% Check the 1st and 2nd order smoothness
d = 5;
cksmooth(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
 [xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
  xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
