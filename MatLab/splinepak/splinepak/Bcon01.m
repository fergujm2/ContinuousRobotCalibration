%  Bcon01 
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE:  Computes a C^0 linear spline interpolant to given data
%   Uses swapping to find a triangulation giving convexity

% Input a triangulation
[n,x,y,nt,TRI] = readtri;
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
figure; triplot(TRI,x,y); axis equal; axis off;

% Sample a test function
z = conf(x,y);

% Evaluate the spline on a grid
d = 1; c = z;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y); ng = 41;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g);  colormap(copper);

% Calculate the error 
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% check the convexity conditions
ckmin = conck01(x,y,v1,v2,v3,ie1,ie2,tril,trir,c);
fprintf('conck = %5.2e\n', ckmin);

% adjust the triangulation by swapping
TRI = conswap(v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,x,y,z);

[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
figure; triplot(TRI,x,y); axis equal; axis off;

% plot the convex interpolating spline 
d = 1; c = z;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y); ng = 41;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g);  colormap(copper);

% Calculate the error 
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% check the convexity conditions
ckmin = conck01(x,y,v1,v2,v3,ie1,ie2,tril,trir,c);
fprintf('conck = %5.2e\n', ckmin);

