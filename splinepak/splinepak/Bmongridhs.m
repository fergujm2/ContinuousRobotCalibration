% Bmongridhs
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a monotone C1 cubic spline interpolating
%  monotone data on a grid, using the type-II triangulation of the grid.

% Create data grid
nx = input('input nx '); ny = input('input ny '); 
x = linspace(0,1,nx); y = linspace(0,1,ny);
z = zeros(nx,ny); zx= zeros(nx,ny); zy= zeros(nx,ny);

% Create type-II triangulation
[tx,ty,TRI] = type2nu(x,y);      %figure; triplot(TRI,tx,ty);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(tx,ty,TRI);

% Sample a test function on the grid
for i = 1:nx
  xi = x(i);
  for j = 1:ny
     b = sigmoider(xi,y(j));
     z(i,j) = b(1); zx(i,j) = b(2); zy(i,j) = b(3);
   end
end

% Compute gradient values at the grid points
ia = input('Gradient options: input 0 for zero, 1 to estimate them, 2 for true ');
if ia == 0
  zx = zeros(nx,ny); zy = zeros(nx,ny); % set gradients to zero
elseif ia == 1
  [zx,zy] = gradgrid(x,y,z);   % Estimate the gradients
end

% Adjust the gradients if desired
ia = input('input 1 to adjust the gradients ');
if ia == 1
  [zx,zy] = hsadj(x,y,z,zx,zy);
end

% Compute the B-coefficients of the interpolating spline
c = cubsib(x,y,z,zx,zy,e1,e2,e3);

% Check the C1 continuity
d = 3; n = nx*ny + (nx-1)*(ny-1);
c1ck(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid 
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y); 
[xg,yg,g] = valspgrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g);  colormap(copper);

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,gx] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,...
    ng,u,xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,gx');  colormap(copper);

% Evaluate the y-derivative on the grid
u = [0,1];
[xg,yg,gy] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,...
    ng,u,xmin,xmax,ymin,ymax);

% Plot the y-derivative
figure; surfl(xg,yg,gy');  colormap(copper);

% Calculate the errors
fn = zeros(ng,ng); fnx = zeros(ng,ng); fny = zeros(ng,ng);
for i = 1:ng
 xgi = xg(i);
 for j = 1:ng
   b = sigmoider(xgi,yg(j));
   fn(i,j) = b(1); fnx(i,j) = b(2); fny(i,j) = b(3);
 end
end

e = fn - g;  emax = max(max(abs(e)));
RMS = sqrt(sum(sum(e.^2))/(ng*ng));
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n', emax,RMS)

e = fnx - gx;  emax = max(max(abs(e)));
RMS = sqrt(sum(sum(e.^2))/(ng*ng));
fprintf('x-Deriv, Maximum error: %5.2e,   RMS = %5.2e\n', emax,RMS)

e = fny - gy;  emax = max(max(abs(e)));
RMS = sqrt(sum(sum(e.^2))/(ng*ng));
fprintf('y-Deriv, Maximum error: %5.2e,   RMS = %5.2e\n', emax,RMS)

% Check if derivatives are positive on the evaluation grid
mx = min(min(gx)); my = min(min(gy));
fprintf('The minimum of the x and y-derivs = %5.2e,  %5.2e\n', mx,my)
