%  B01disk
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate data at the vertices of a triangulation 
%    of a disk with a C0 linear spline

% Read in and plot the triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
xmin = min(x); xmax = max(x); ymin = min(x); ymax = max(x);

% Sample a function at the vertices of the triangulation
r = @(x,y) sqrt(x.^2 + y.^2);
f = @(x,y) cos(pi*r(x,y));
c = f(x,y);

% Render the surface on the smallest enclosing rectangle
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
xmin = min(x); xmax = max(x); ymin = min(x); ymax = max(x);

% Evaluate the spline on a grid of size ng
ng = 51; d = 1;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
