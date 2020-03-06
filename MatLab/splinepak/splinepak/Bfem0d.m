% Bfem0d
% SplinePAK: Copyright Larry Schumaker 2014
% Solve a 2nd Order BVP with Dirichlet boundary conditions
%   using the S0d spline space

% Example 9.11
kappa = @(x,y) exp(x+y);
u = @(x,y) -sin(4*x) - sin(4*y);
f = @(x,y) exp(x+y).*(-16*sin(4*x) - 16*sin(4*y) + 4*cos(4*x) + 4*cos(4*y));
g = u;

% Example 9.12  
%kappa = @(x,y) 1;
%u = @(x,y) sin(x.^2+y.^2) + .1*sin(25*(x.^2+y.^2));
%f = @(x,y) -4.*cos(x.^2+y.^2) + 4*(x.^2+y.^2).*sin(x.^2+y.^2) ... 
%  -10*cos(25*(x.^2+y.^2)) + 250*(x.^2+y.^2).*sin(25*(x.^2+y.^2));
%g = u;

% Example 9.14 -- see also Gockenbach: p. 94  -- on disk
%kappa = @(x,y) 1;
%f = @(x,y) 12.*x.*y;
%u = @(x,y) (x.^2 + y.^2 <= 1).*(x.*y - y.*x.^3 - x.*y.^3);

% Input a triangulation
[n,x,y,nt,TRI] = readtri;     %triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Choose the degree of the spline
d = input('input the spline degree d =  ');

if d == 1
  [c,M,t1,t2] = fem01(x,y,v1,v2,v3,e1,e2,e3,ie1,area,bdy,kappa,f,g);
else
  [c,M,t1,t2,cb] = fem0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,...
     tril,trir,area,bdy,kappa,f,g);  % fastest version
end
fprintf('times to assemble and solve %g %g \n',t1,t2);

%tic; fprintf('condition number %5.2e \n',cond(full(M))); toc
%size(M)

% Evaluate the spline on a grid
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,gv] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Compute the max and RMS errors
e = errg(xg,yg,gv,u);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline
figure; surfl(xg,yg,gv');  colormap(copper);
return

fprintf('size M %g,  condition number %g \n',length(M),condest(M));
