% SPct
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered Hermite data on the sphere using a Clough-Tocher spline

d = 3;
% Input the data points and a triangulation
[no,x,y,z,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] = slists(x,y,z,TRIo);
neo = length(ie1o); w = zeros(no+neo,1); grad = zeros(no+neo,3);

% Calculate frame vectors at the vertices
[u,uw] = sframe(x,y,z);

% Calculate frame vectors at the midpoints of each edge
[xm,ym,zm,ue] = sframemid(x,y,z,ie1o,ie2o);

% Get function values and gradients at the data points
nf = input('input nf ');
[w,grad] = sfun(nf,x,y,z);

% Compute frame derivatives at each vertex
w1 = zeros(1,no);   w2 = zeros(1,no);   we = zeros(1,neo);
for i = 1:no
  w1(i) = u(i,:)*grad(i,:)';
  w2(i) = uw(i,:)*grad(i,:)';
end

% Compute the direction vector and cross deriv at each midpoint
for i = 1:neo
 i1 = ie1o(i); i2 = ie2o(i);
 [wm,gradm] = sfun(nf,xm(i),ym(i),zm(i));
  we(i) = ue(i,:)*gradm';
end

% Find the B-coefficients of the spline
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  sphct(x,y,z,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,...
    w,u,uw,w1,w2,ue,we);
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
