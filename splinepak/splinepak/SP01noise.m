% SP01noise 
% SplinePAK: Copyright Larry Schumaker 2014
% SPAK: Copyright Larry Schumaker 2012
% Interpolates scattered data on the sphere with a C^0 linear spherical spline
% VERSION: with noise

d = 1;
% Read the data and a triangulation
[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] =  slists(x,y,z,TRI);

% Sample a test function at the data points
nf = input('input nf ');
eps = input('input eps ');
vn = readnoise(n);
c = sfun(nf,x,y,z) + eps*vn;

% Evaluate the spline on the triangulation stri6
tic
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');
toc

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
axis vis3d; axis equal tight off;  rotate3d on;
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);

% Evaluate on a sptri7 for calculating errors
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));



