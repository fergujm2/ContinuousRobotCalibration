% SPlsqct
% SplinePAK: Copyright Larry Schumaker 2014
% Find a least-squares fit of scattered data on the sphere
%   using a Clough-Tocher spline

% Input a triangulation
[no,xo,yo,zo,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] = ...
     slists(xo,yo,zo,TRIo);

% Find the transformation matrix
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A,dof]...
  = smdsct(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
d = 3;  
toc

% Input data points
[nd,xd,yd,zd] = sreadpts;

% Sample a test function at the data points (and add noise)
nf = input('input nf ');
eps = input('input eps for noise ');
rd = readnoise(nd);
wd = sfun(nf,xd,yd,zd) + eps*rd;
wt = ones(nd,1); % Set the weighs to one

% Find the coefs of the LSQ Clough-Tocher spline
[c,ta,ts,M] = slsq(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,xd,yd,zd,wd,wt,A);
fprintf('time assemble = %g, solve = %g\n', ta,ts);

% Check C1 smoothness
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on sptri6 
tic
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');
toc

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
axis vis3d; axis equal tight off;  hidden on; rotate3d on;

% Evaluate on sptri7 for error computation
tic
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');
toc

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));
