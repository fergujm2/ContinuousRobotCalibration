% SPct  10/2/13
% SPAK: Copyright Larry Schumaker 2012
% Interpolate scattered Hermite data on the sphere using a Clough-Tocher spline

d = 3;
% Input the data points and a triangulation
[no,x,y,z,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] = slists(x,y,z,TRIo);
neo = length(ie1o); w = zeros(no+neo,1); grad = zeros(no+neo,3);

% Get function values and gradients at the data points
nf = input('input nf ');
[w,grad] = sfun(nf,x,y,z);

% Compute the gradients at the midpoints of the edges
for i = 1:neo
  i1 = ie1o(i); i2 = ie2o(i);
  [xm,ym,zm] = unit((x(i1)+x(i2))/2,(y(i1)+y(i2))/2,(z(i1)+z(i2))/2);
  [w(i+no),grad(i+no,:)] = sfun(nf,xm,ym,zm);
end

% Find the B-coefficients of the spline
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  sphctgrad(x,y,z,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,w,grad);
toc

% Check c1 continuity
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
