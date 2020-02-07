% SPlsq15 
% SplinePAK: Copyright Larry Schumaker 2014
% Find a least-squares fit to scattered data on the sphere
%   using a C1 quintic spline

% Read in a triangulation
[n,x,y,z,nt,TRI] = sreadtri;

% Compute the triangulation lists and find the transformation matrix
tic
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] = slists(x,y,z,TRI);
[A,dof] = smds15(x,y,z,v1,v2,v3,e1,e2,e3,ie1); d = 5;  
toc

% Read in the data points
[nd,xd,yd,zd] = sreadpts;

% Sample a test function at the data points
nf = input('input nf ');
wd = sfun(nf,xd,yd,zd);
noise = readnoise(nd);
eps = input('input eps for noise ');
wd = wd + eps*noise;
wt = ones(nd,1); % Set the weights to one

% find the coefficients of the LSQ spline
[c,ta,ts,M] = slsq(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,xd,yd,zd,wd,wt,A);
fprintf('time assemble = %g, solve = %g\n', ta,ts);

% Check C1
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

% Evaluate the spline on sptri6 
tic
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');
toc

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
axis vis3d; axis equal tight off;  hidden on; rotate3d on;

% Evaluate on \sptri7 for error computation
tic
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');
toc

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));
