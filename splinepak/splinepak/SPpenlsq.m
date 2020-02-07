%  SPpenlsq
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data on the sphere using a 
%   penalized least-squares Powell-Sabin spline or Clough-Tocher spline

% Input a triangulation
[no,xo,yo,zo,ntro,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] =  slists(xo,yo,zo,TRIo);

% Input data points
[nd,xd,yd,zd] = sreadpts;

% Choose method and compute the transformation matrix
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A,dof]...
  = smdsps(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
d = 2;

% sample a test function at the data points
nf = input('input nf ');
wd = sfun(nf,xd,yd,zd);
wt = ones(no,1); % set weights

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);  wd = wd + eps*rv;
end

% Input the parameter lambda
lam = input('input lambda ');

% Compute the coefs of the penalized least-squares spline
c = spenlsq(d,x,y,z,xd,yd,zd,wd,wt,v1,v2,v3,e1,e2,e3,ie1,A,lam);

% Check C1 smoothness
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c)

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
