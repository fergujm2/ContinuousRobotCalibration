% Bposlsqps 
% SplinePAK: Copyright Larry Schumaker 2014
% Fit scattered data with a nonnegative C1 Powell-Sabin spline
%   using least-squares 

% Read in a triangulation
[no,xo,yo,nto,TRIo] = readtri;   

% Alternate: Create a Delaunay triangulation
%[no,xo,yo] = readxy; TRIo = delaunay(xo,yo); figure; triplot(TRIo,xo,yo);

% Compute the triangulation information
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRIo);

% Input random data points
nd = input('input number of data points nd = ');
[xd,yd] = randpts(nd);

% Sample a test function at the data points
zd = hill(xd,yd); wd = ones(nd,1);

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Compute the Powell-Sabin refinement and the transformation matrix A
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);

% Plot the refined triangulation
%TRI = [v1,v2,v3]; figure; triplot(TRI,x,y);

% Compute the B-coefficients of the least-squares spline
d = 2;
[c,G,t1,t2] = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);
fprintf('time to assemble and solve %g + %g \n',t1,t2);
%fprintf('cond normal eqn  %g\n',cond(full(G)));

% Check C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors on the grid
e = errg(xg,yg,g,@hill);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check the min value of the spline on the grid
fprintf('Minimum of the spline: %5.2e\n', min(min(g)))

%%%%%% Nonnegative adjustment

% Make the vertex values nonnegative
z = max(c(1:no),0);

% Estimate and adjust the gradients 
[zx,zy] = gradps(no,nto,x,y,ie1o,ie2o,eadjo,adjstarto,c);
[zx,zy] = adjgradpspos(xo,yo,z,ie1o,ie2o,zx,zy);

% Compute all B-coefficients
cp = A*[z;zx;zy];

% Check C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,cp);

% Evaluate the new spline on the grid
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,cp,ng,xmin,xmax,ymin,ymax);

% Plot the nonnegative spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors on the grid
e = errg(xg,yg,g,@hill);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check the minimum value of the spline on the grid
fprintf('Minimum of the spline: %5.2e\n', min(min(g)))
