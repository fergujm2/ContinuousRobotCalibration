% Bfem0disk
% SplinePAK: Copyright Larry Schumaker 2014
% Solve 2nd Order BVP with Dirichlet boundary conditions
%   Using S0d spline space on a DISK

% Set up the BVP for EXAMPLE 9.14
kappa = @(x,y) 1;
f = @(x,y) 20*x.^3.*y.^2 +6*(x.*y.^4 - x.*y.^2) + 12*y.^2.*x.^3 ...
    + 2*(x.^5 - x.^3);
u = @(x,y) (1-x.^2 - y.^2).*(x.^3.*y.^2);
g = u;

% Input a triangulation
[n,x,y,nt,TRI] = readtri;     %triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

d = input('input degree of the spline d = ');

%  Compute the coefs of the spline 
if d == 1
  [c,M,t1,t2] = fem01(x,y,v1,v2,v3,e1,e2,e3,ie1,area,bdy,kappa,f,u);
else
  [c,M,t1,t2] = fem0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,...
     tril,trir,area,bdy,kappa,f,g);  
end
fprintf('times to assemble and solve %g %g \n',t1,t2);
%fprintf('size M %g,  condition number %5.2e \n',length(M),cond(full(M)));

% Evaluate the spline on an enclosing rectangle
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Calculate the error  at grid points inside the triangulation
e = errg(xg,yg,g,u);
fprintf('Maximum error: %5.2e,  RMS = %5.2e\n', norm(e,inf),erms(e));

% Replace the NaN's in g by zeros for a smoother looking plot
b = find(isnan(g));  g(b) = zeros(length(b),1);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);
