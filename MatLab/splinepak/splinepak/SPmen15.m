% SPmen15
% SplinePAK: Copyright Larry Schumaker 2014
% Find minimal energy spline interpolating scattered data on the sphere

% Read a spherical triangulation
[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] = slists(x,y,z,TRI);

% Find the transformation matrix
t0 = cputime;
[A,dof] = smds15(x,y,z,v1,v2,v3,e1,e2,e3,ie1); d = 5;
tm = cputime - t0;

% Sample a test function at the data points
nf = input('input nf ');
w = sfun(nf,x,y,z);

% Compute the coefs of the minimal energy spline
[c,Msys,t1,t2] = smen(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,w,A);
fprintf('Time: mds, assemble and solve %g %g %g \n',tm,t1,t2);
fprintf('condition number %g \n',cond(Msys));

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
