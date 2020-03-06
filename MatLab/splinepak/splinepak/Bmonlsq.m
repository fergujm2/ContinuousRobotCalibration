%Bmonlsq 
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a monotone C1 cubic spline based on
%  noisy data at scattered data points. Uses a C1 Powell-Sabin least-squares
%  fit in the first stage, and a C1 cubic spline on a type-II triangulation
%  in the second stage


% Read in some data points
%[nd,xd,yd] = readxy;

% For random points
nd = input('input nd ');
[xd,yd] = randxy(nd);

% Sample a test function at the data points
f = @(x,y) sigmoid(x,y);
zd = f(xd,yd);

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Read a triangulation for the spline
[no,xo,yo,nto,TRIo] = readtri;   figure; triplot(TRIo,xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRIo);

% Compute the Powell-Sabin lsq spline
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,...
  dof,A] =  mdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,...
  ie1o,ie2o,trilo,triro,adjstarto,eadjo,bdyo);
TRI = [v1,v2,v3]; figure; triplot(TRI,x,y);
d = 2;  wd = ones(nd,1);
c = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);

% Check C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the first stage spline on a grid
ng = 51; 
xmin = min(xd);; xmax = max(xd); ymin = min(yd);  ymax = max(yd);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the first stage spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative on the grid
ux = [1,0];
[xg,yg,gx] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,ux,...
   xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,gx');  colormap(copper);

% Evaluate the y-derivative on the grid
uy = [0,1];
[xg,yg,gy] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,uy,...
    xmin,xmax,ymin,ymax);

% Plot the y-derivative
figure; surfl(xg,yg,gy');  colormap(copper);

% Check if derivatives are positive on the evaluation grid
mx = min(min(gx)); my = min(min(gy));
fprintf('minimum of x and y derivatives of 1st stage = %5.2e %5.2e\n',mx,my);

% Choose knots for a type-2 triangulation on the rectangle
nx = input('input number of x-knot lines nx =  '); 
ny = input('input number of y-knot lines ny =  ');
x2 = linspace(xmin,xmax,nx); y2 = linspace(ymin,ymax,ny);

% Evaluate the first stage spline and its derivatives on the type-2 grid
z = valspgrida(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,x2,y2);
zx = valspdergrida(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ux,x2,y2);
zy = valspdergrida(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,uy,x2,y2);

% Adjust z-values 
z = monzadja(x2,y2,z);

% Adjust the gradients
[zx,zy] = hsadj(x2,y2,z,zx,zy);

% Compute the B-coefficients of the 2nd stage C1 cubic spline interpolant
[tx,ty,TRI] = type2nu(x2,y2);
figure; triplot(TRI,tx,ty);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(tx,ty,TRI);
c = cubsib(x2,y2,z,zx,zy,e1,e2,e3);

% Check the C1 continuity
d = 3;  c1ck(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate on a grid
ng = 51; 
[xg,yg,g] = valspgrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the final interpolating spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the error
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative
u = [1,0];
[xg,yg,gx] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,...
   ng,u,xmin,xmax,ymin,ymax);

% Plot the x-derivative
%figure; surfl(xg,yg,gx');  colormap(copper);

% Evaluate the y-derivative
u = [0,1];
[xg,yg,gy] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,...
   ng,u,xmin,xmax,ymin,ymax);

% Plot the y-derivative
figure; surfl(xg,yg,gy');  colormap(copper);

% check if derivatives are positive on the evaluation grid
mx = min(min(gx)); my = min(min(gy));
fprintf('minimum of x and y derivatives of 2nd stage = %5.2e %5.2e\n',mx,my);
