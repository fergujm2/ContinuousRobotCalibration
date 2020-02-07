% SPscat03
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates scattered data on the sphere using  S^0_d
% This is a local method

% Read in data and a triangulation
[n,x,y,z,nt,TRI] = sreadtri; d = 3;
tic
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] =  slists(x,y,z,TRI);
toc

% Sample a test function at the data points
nf = input('input nf ');
w = sfun(nf,x,y,z);

% Compute the spline coefs
tic
m = 20;
c = sloc03(x,y,z,w,v1,v2,v3,e1,e2,e3,ie1,ie2,m);
toc

% Check the C1 smoothness
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

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
