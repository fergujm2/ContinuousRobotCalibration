% Brendspgrid
% SplinePAK: Copyright Larry Schumaker 2014
% Render a spline on a general polygonal domain using valspgrid

% Read in a triangulation
[nv,x,y,nt,TRI] = readtri; triplot(TRI,x,y);

% Compute the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Choose a degree and compute the number of coefs of a spline
d = input('input degree of the spline d = ');
nc = nv + (d-1)*ne + choose(d-1,2)*nt

% Set one coef to one and the rest to zero
nset = input('input number of coefficient to set, nset = ');
c = zeros(nc,1);  c(nset) = 1;

% Evaluate the spline on a rectangular grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[gx,gy,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(gx,gy,g'); colormap(copper);



