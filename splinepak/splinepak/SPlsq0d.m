% SPlsq0d
% SplinePAK: Copyright Larry Schumaker 2014
% Computes a least-squares fit of scattered data on the sphere using S^0_d

% Read a triangulation
[n,x,y,z,nt,TRI] = sreadtri;
tic
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] =  slists(x,y,z,TRI);
toc

% Read data points
[nd,xd,yd,zd] = sreadpts;

% Sample a test function at the data points and possibly add noise
nf = input('input nf ');
nv = readnoise(nd);
eps = input('input size of noise ');
fd = sfun(nf,xd,yd,zd);
wd = fd + eps*nv; 

% Set weights to ones
nd = length(xd); wt = ones(nd,1);

% Compute the least-squares spline from S0d
d = input('input d ');
tic
[c,G] = slsq0d(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,xd,yd,zd,wd,wt);
toc

% Check c1 continuity
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on sptri6 
%tic
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');
%toc

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
axis vis3d; axis equal tight off;  hidden on; rotate3d on;

% Evaluate on sptri7 for error computation
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));
