% TPscatpsdat
% SplinePAK: Copyright Larry Schumaker 2014
% Almost interpolate scattered data with a tensor product spline
% First stage uses a Powell-Sabin interpolant with estimated derivatives
% Version of TPscatps, but reads the z values instead of sampling a test fn
%   and uses the Delaunay triangulation in the 1st stage

% Read in the data
[no,xo,yo,zo] = readxyz;

% Find the Delaunay triangulation
tic
TRI = delaunay(xo,yo);        %triplot(TRI,x,y);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRI);
toc

% 1st stage Estimate derivatives and construct the Powell-Sabin interpolant
m = 20; de = 3;
tic
[zxo,zyo] =  derestlsq(xo,yo,zo,adjstarto,vadjo,de,m);
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
   ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,zo,zxo,zyo);
toc

% Convert this spline to a tensor-product spline
nx = input(' input number of x-knot lines '); 
ny = input(' input number of y-knot lines '); 
d = 2;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);
tic
[xe,ye,ctp] = sp2tp(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,xmin,xmax,ymin,ymax,nx,ny);
toc

% Check how close the TP comes to interpolating the original data
e = zeros(1,no); mx = nx+1; my = ny+1;
for i = 1:no;
 xi = xo(i); yi = yo(i);
  l = findinterval (mx,xe,xi);
  lw = findinterval (my,ye,yi);
  e(i) = valtp(3,3,xe,ye,ctp,l,lw,xi,yi) -  zo(i);
end

fprintf('max interp err = %5.2e rms interp err =  %5.2e \n',...
    norm(e,inf),erms(e));

% Evaluate the TP spline on a rectangular grid
x1 = input('input x1 '); x2 = input('input x2 ');
y1 = input('input y1 '); y2 = input('input y2 ');
ng = 51; d = 3;  nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,d,xe,ye,ctp,nu,mu,ng,ng,x1,x2,y1,y2);

% Plot the TP spline
figure; surfl(xg,yg,g'); colormap(copper);
