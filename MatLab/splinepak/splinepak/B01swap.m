% B01swap 
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates a given function with a C0 Linear Spline
% adjusts the triangulation to get best LSQ error on a grid of points

% Input a triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);  axis equal; axis off;
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);

% Sample the test function
%f = @(x,y) franke2(x,y);
%f = @(x,y) (y  >= 1-x).*(y+x-1);
f = @(x,y) (x.^2 +y.^2 >1).*(x.^2+y.^2-1).^3;
z = f(x,y); c = z;

% Evaluate the spline on a grid
ng = 51; d = 1; 
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g'); colormap(copper); 

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Do swapping to get a better triangulation in terms of error
[v1,v2,v3] = swap01(v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,x,y,z,ng,f);
figure; triplot([v1,v2,v3],x,y); axis equal; axis off;

% Evaluate the new spline on the grid
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

