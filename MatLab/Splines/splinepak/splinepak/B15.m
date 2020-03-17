%  B15
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data with a C1 quintic Argyris spline

% Read in the triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);

% Compute the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

%  Compute derivatives at the vertices
der = franke2d(x,y);

%  Compute cross derivatives at the midpoints of edges
x1 = x(ie1); x2 = x(ie2); y1 = y(ie1); y2 = y(ie2);
xmid = (x1+x2)./2; ymid = (y1+y2)./2;
dmid = franke2d(xmid,ymid);
cross = zeros(ne,1);
for i = 1:ne
  [dx,dy] =  ucross(x1(i),y1(i),x2(i),y2(i));
  cross(i) = dx*dmid(i,2) + dy*dmid(i,3);
end

%  Compute the coefs of the spline
tic
c =  arg15(x,y,v1,v2,v3,e1,e2,e3,ie1,der,cross);
toc

% Check the C1 smoothness
d = 5;
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on a grid
ng = 51;
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
toc

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,gx] =  valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,gx');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,gx,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
