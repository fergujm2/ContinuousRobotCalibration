%  B01dat  
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate data at the vertices of a triangulation
%    with a C^0 linear spline

% Input data 
[n,x,y,z] = readxyz;

% Compute Delaunay triangulation
tic
 TRI = delaunay(x,y);
toc

% Find triangulation lists
tic
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
toc

% Evaluate the spline on a rectangle
fprintf('input rectangle ');
x1 = input('input x1 ');  x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');

tic
ng = 81; d = 1;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,z,ng,x1,x2,y1,y2);
toc

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper); axis equal;
