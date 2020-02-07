% Bmen15
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate scattered data with a minimal energy
%   C1 Argyris quintic spline using a minimal determining set for the space

[n,x,y,nt,TRI] = readtri;     %triplot(TRI,x,y);
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);

[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Sample a function at the vertices of the triangulation
f = @(x,y) franke2(x,y);
z = f(x,y);

% Find a minimal determining set and the transformation matrix
to = cputime;
A = mds15(x,y,v1,v2,v3,e1,e2,e3,...
   ie1,ie2,tril,trir,adjstart,eadj,tstart,tadj,bdy);
tm = cputime - to;

d = 5;
% Solve for the coefficients of the minimal energy spline
[c,M2,t1,t2] = men15(x,y,z,v1,v2,v3,e1,e2,e3,ie1,area,A);
fprintf('times: mds, assemble, and solve %g %g %g \n',tm,t1,t2);

% Print the condition number
%fprintf('condition number %g \n',cond(full(M2)));

% Check the C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Render the spline
ng = 51;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('Error: emax = %5.2e,   RMS = %5.2e\n', norm(e,inf),erms(e));

% Plot the x-derivative
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));

