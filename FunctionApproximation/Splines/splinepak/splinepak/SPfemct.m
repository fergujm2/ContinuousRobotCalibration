% SPfemct
% SplinePAK: Copyright Larry Schumaker 2014
% Solve a PDE on the sphere with a spherical Clough-Tocher cubic spline


os = input('input omega^2 ');
% Example 11.54
f = @(x,y,z) (12 + os)*(y.^3 +z.^3) -6*(y+z);
true = @(x,y,z) y.^3 +z.^3;

% 5th degree
%f = @(x,y,z) (30 + os)*x.^2.*z.^3 - 2*z.^3 - 6*x.^2.*z;
%true = @(x,y,z) x.^2.*z.^3;

% Example 11.55
%f = @(x,y,z) (os +x.^2 + 2*x-1).*exp(x);
%true = @(x,y,z) exp(x);

% Read a triangulation
tic
[no,xo,yo,zo,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] =  slists(xo,yo,zo,TRIo);
neo = length(ie1o);
toc

% Find the Clough-Tocher refinement and the transformation matrix
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A,dof] = ...
  smdsct(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
toc
d = 3;

% Find the B-coefficients
[c,M] = sfem(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,A,os,f);
cond(M)

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

% error calc
e = true(xp,yp,zp) - g;
fprintf('emax = %5.2e RMS = %5.2e\n', norm(e,inf),erms(e));
