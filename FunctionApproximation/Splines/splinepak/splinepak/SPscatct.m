% SPscatct
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data on the sphere with a Clough-Tocher spline
%   using estimated derivatives

d = 3;  eps = 3;
% Input a triangulation
[no,x,y,z,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] = slists(x,y,z,TRIo);

% Calculate the frame vectors at the vertices
[u,uw] = sframe(x,y,z);

% Calculate frame vectors at the midpoints of each edge
[xm,ym,zm,ue] = sframemid(x,y,z,ie1o,ie2o);

% Sample a test function at the data points
nf = input('input nf ');
w = sfun(nf,x,y,z);

% Estimate the frame derivatives at the data points
tic
de = 3; m = 20;
[w1,w2] = sderest(x,y,z,w,v1o,v2o,v3o,ie1o,ie2o,u,uw,de,m);
toc

% Estimate the frame derivatives at the midpoints of edges
we = sderestmid(x,y,z,w,v1o,v2o,v3o,ie1o,ie2o,trilo,triro,...
  xm,ym,zm,ue,de,m);

% Compute the coefs of the interpolating spline
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  sphct(x,y,z,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,...
    w,u,uw,w1,w2,ue,we);
toc

% Check C1 smoothness
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

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

return
% Evaluate on \sptri7 for error computation
tic
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');
toc

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));
