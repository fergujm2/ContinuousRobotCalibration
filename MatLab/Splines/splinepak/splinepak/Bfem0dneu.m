% Bfem0neu
% SplinePAK: Copyright Larry Schumaker 2014
% Solve a 2nd Order BVP with pure Neumann boundary conditions
%   using the spline space S0d

% Example 9.16
kappa = @(x,y) exp(x+y);
u = @(x,y) -sin(4*x) - sin(4*y);
f = @(x,y) exp(x+y).*(-16*sin(4*x) - 16*sin(4*y) + 4*cos(4*x) + 4*cos(4*y));
ux = @(x,y) -4*cos(4*x); uy = @(x,y) -4*cos(4*y);
h = @(x,y) (ux(x,y).*(-(x==0) + (x==1)) + uy(x,y).*(-(y==0) + (y==1))) ...
   .*kappa(x,y);

% Input a triangulation
[n,x,y,nt,TRI] = readtri;    %triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Choose a degree
d = input('input the degree of the spline  d = ');

% Compute the coefs of the Rayleigh-Ritz spline
[c,M,t1,t2] = fem0dneu(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,...
   tril,trir,area,kappa,f,h);
fprintf('times to assemble and solve %g %g \n',t1,t2);

% Evaluate the spline on an enclosing rectangle
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Compute the max and RMS errors
e = errg(xg,yg,g,u);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

fprintf('size M %g,  condition number %5.2e \n',length(M),condest(M));
