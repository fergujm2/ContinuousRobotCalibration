% Bmonscatai
% SplinePAK: Copyright Larry Schumaker 2014
% Find a C1 cubic monotone spline that almost interpolates given
%  monotone scattered data based on the two-stage method of Sect. 8.3.2.3
%  where the first stage is a Powell-Sabin minimal energy spline
% Version: almost interpolates

% Read in the data points 
[no,xo,yo] = readxy;
xmin = min(xo); xmax = max(xo); ymin = min(yo); ymax = max(yo);

% Create the Delaunay triangulation of these points
TRI = delaunay(xo,yo);

%Sample a test function at the data points
f = @(x,y) sigmoid(x,y);
zo = f(xo,yo);

% Fit minimal energy Powell-Sabin spline to data
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRI);
[xp,yp,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
[c,M,t1,t2] = menps(v1o,v2o,v3o,xp,yp,zo, ...
   v1,v2,v3,e1,e2,e3,ie1,A);

% Evaluate this 1st stage spline on a grid
ng = 51; d = 2;
[xg,yg,g] = valspgrid(d,xp,yp,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the 1st stage spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n', norm(e,inf),erms(e));

% STAGE 2: 
% create a grid and evaluate the spline on that grid
nx = input('input number of x-grid lines nx =  ');   
ny = input('input number of y-grid lines ny = ');  
[x,y,z] = valspgridxy(d,xp,yp,v1,v2,v3,e1,e2,e3,ie1,c,nx,ny,...
    xmin,xmax,ymin,ymax);

% construct the type-II triangulation with grid lines given by x and y
[tx,ty,TRI] = type2nu(x,y); 
figure; triplot(TRI,tx,ty);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(tx,ty,TRI);

% evaluate the gradients on the grid
[zx,zy] = gradgrid(x,y,z);

% adjust the grid values to be monotone consistent
z = monzadj(x,y,xo,yo,zo,z);

% adjust the gradients
[zx,zy] = hsadj(x,y,z,zx,zy);

% compute the B-coefficients of the C^1 cubic spline interpolant
c = cubsib(x,y,z,zx,zy,e1,e2,e3);

% check the C1 continuity
d = 3;  c1ck(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate this cubic spline on a grid
ng = 51; 
[xg,yg,g] = valspgrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Evaluate the x-derivative
u = [1,0];
[xg,yg,gx] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,gx');  colormap(copper);

%  Evaluate the y-derivative on the grid
u = [0,1];
[xg,yg,gy] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);

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

e = fn - g;  e = reshape(e,1,ng*ng);
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n', norm(e,inf),erms(e))

e = fnx - gx;  e = reshape(e,1,ng*ng);
fprintf('x-Deriv, Maximum error: %5.2e,  RMS = %5.2e\n', norm(e,inf),erms(e))

e = fny - gy;  e = reshape(e,1,ng*ng);
fprintf('y-Deriv, Maximum error: %5.2e, RMS = %5.2e\n', norm(e,inf),erms(e))

% Check if derivatives are positive on the evaluation grid
mx = min(min(gx)); my = min(min(gy));
fprintf('The minimum of the x and y-derivs = %5.2e,  %5.2e\n', mx,my)

% Check how close the spline is to interpolating the data
e = zo - valsp(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,xo,yo);
fprintf('The max and RMS discrepencies  = %5.2e, %5.2e\n',norm(e,inf),erms(e));
