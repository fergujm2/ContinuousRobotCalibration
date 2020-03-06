% SPmenps
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data on the sphere with a minimal energy 
%   Powell-Sabin spline

% Read in the data and a triangulation
[no,xo,yo,zo,ntro,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] =  slists(xo,yo,zo,TRIo);

d = 2;  
% Sample a test function at the data points
nf = input('input nf ');
w = sfun(nf,xo,yo,zo);

% Compute the transformation matrix
to = cputime;
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A,dof]...
  = smdsps(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
tm = cputime - to;

% Compute the coefficients of the minimal energy spline
n = length(x); ne = length(ie1); nt = length(v1);
[c,Msys,t1,t2] = smen(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,w,A); %FASTEST
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

% Evaluate on sptri7 for error computation
tic
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');
toc

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));
