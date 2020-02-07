% SPscatlocct
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates scattered data on the sphere with a Clough-Tocher spline
% This is an adhoc local method not using estimated derivatives
% Instead it adjusts the coefs of a spline in S03

% Input a triangulation
[no,xo,yo,zo,nto,TRIo] = sreadtri; d = 3;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] =  slists(xo,yo,zo,TRIo);

% Sample a test function at the data points
nf = input('input nf ');
w = sfun(nf,xo,yo,zo);

% Compute the coeffients of the Clough-Tocher interpolant
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
    slocct(xo,yo,zo,w,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
toc

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

% Compute the error
err = serr(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,nf);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));
