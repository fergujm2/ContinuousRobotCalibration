% TPscatmenps 
% SplinePAK: Copyright Larry Schumaker 2014
% Almost interpolate scattered data with a tensor product spline
% First stage uses a minimal energy Powell-Sabin interpolant

% Read in a triangulation
[no,xo,yo,nto,TRI] = readtri;
%triplot(TRI,x,y);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRI);

% Sample a test function at the vertices
f = @(x,y) franke2(x,y);
zo = f(xo,yo);

% 1st stage -- Compute the minimal energy Powell-Sabin interpolant
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
[c,M,t1,t2] = menps(v1o,v2o,v3o,x,y,zo, ...
   v1,v2,v3,e1,e2,e3,ie1,A);

% Convert this spline to a tensor-product spline
nx = input(' input number of x-knot lines '); 
ny = input(' input number of y-knot lines '); 
d = 2;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);
[xe,ye,ctp] = sp2tp(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,xmin,xmax,ymin,ymax,nx,ny);

% Evaluate the spline on a grid
ng = 51; d = 3; nu = 0; mu = 0;
%[xg,yg,g] =  valtpgrid(d,d,mx,my,xe,ye,ctp,nu,mu,ng,ng,xmin,xmax,ymin,ymax);
[xg,yg,g] = valtpgrid(d,d,xe,ye,ctp,nu,mu,ng,ng,xmin,xmax,ymin,ymax);

% Plot the tensor-product spline
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS error on the grid
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check how close it is to interpolating at the original scattered data points
e = zeros(1,no); mx = nx+2; my = ny+2; nu = 0; mu = 0;
for i = 1:no;
 xi = xo(i); yi = yo(i);
  l = findinterval (mx,xe,xi);
  lw = findinterval (my,ye,yi);
  e(i) = valtp(3,3,xe,ye,ctp,l,lw,xi,yi) -  zo(i);
end

fprintf('max interp err = %5.2e rms interp err =  %5.2e \n',norm(e,inf),erms(e));

