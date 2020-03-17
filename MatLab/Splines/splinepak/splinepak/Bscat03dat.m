% Bscat03dat
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a C0 cubic spline
% Computes the coefficients on each triangle by local least-squares

% Input data 
[n,x,y,z] = readxyz;

% Compute Delaunay triangulation
 TRI = delaunay(x,y);

% Calculate the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the coefficients of the interpolating spline
tic
c = scat03(x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,adjstart,vadj);
toc

% Evaluate the spline on a rectangle
x1 = input('input x1 ');  x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');

tic
ng = 81; d = 3;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,x1,x2,y1,y2);
toc

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;

