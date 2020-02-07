% SPctder 
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates scattered Hermite data on the sphere with a Clough-Tocher spline
% Version of SPct which computes errors for derivatives

d = 3;
% Input the data points and a triangulation
[no,x,y,z,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] = slists(x,y,z,TRIo);
neo = length(ie1o); w = zeros(no+neo,1); grad = zeros(no+neo,3);

% Calculate frame vectors
[u,uw] = sframe(x,y,z);

% Find function values and gradients at the data points
nf = input('input nf ');
[w,grad] = sfun(nf,x,y,z);

% Compute frame derivatives at each vertex
w1 = zeros(1,no); w2 = zeros(1,no);
ue = zeros(neo,3); we = zeros(1,neo);
for i = 1:no
  w1(i) = u(i,:)*grad(i,:)';
  w2(i) = uw(i,:)*grad(i,:)';
end

% Compute the direction vector and cross deriv at each midpoint
for i = 1:neo
 i1 = ie1o(i); i2 = ie2o(i);
 [xm,ym,zm] = unit(x(i1)+x(i2),y(i1)+y(i2),z(i1)+z(i2));
 [xt,yt,zt] = stangent(xm,ym,zm,x(i2),y(i2),z(i2));
 [xc,yc,zc] = scross(xm,ym,zm,xt,yt,zt);
  ue(i,:) = [xc,yc,zc];
 [wm,gradm] = sfun(nf,xm,ym,zm);
  we(i) = [xc yc zc]*gradm';
end

% Find the B-coefficients of the spline
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c] = ...
  sphct(x,y,z,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,...
    w,u,uw,w1,w2,ue,we);
toc

% Check C1 continuity
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on sptri6 
tic
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');
toc

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
axis vis3d; axis equal tight off;  hidden on; rotate3d on;

% Compute errors in the derivatives for random directions at each vertex
%   of sptri7
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');

n = length(x); ne = length(ie1); nt = length(v1); np = length(xp);
% Read random points to use in getting random direction vectors
fid = fopen('randpts.dat');
rp = fscanf(fid,'%g',[np,3]); rp = .25*rp;

[it,b1,b2,b3] = sfindtri(x,y,z,v1,v2,v3,xp,yp,zp);

% Compute derivatives in random directions at points of vr7
e = zeros(1,np);
tic
for i = 1:np
  k = it(i); 
  x0 = xp(i); y0 = yp(i); z0 = zp(i);
  v1i = v1(k); v2i = v2(k); v3i = v3(k);
  co = getco(d,k,n,v1,v2,v3,e1,e2,e3,ie1,c);
  x1 = x0 + rp(i,1); y1 = y0 + rp(i,2); z1 = z0+ rp(i,3);  % perturb slightly
  [x1,y1,z1] = unit(x1,y1,z1);
  [u1,u2,u3] = stangent(x0,y0,z0,x1,y1,z1); u= [u1;u2;u3];
  [d1,d2,d3] = sbco(x(v1i),y(v1i),z(v1i),x(v2i),y(v2i),z(v2i),...
      x(v3i),y(v3i),z(v3i),u1,u2,u3);
  valder = decastder (d,1,b1(i),b2(i),b3(i),d1,d2,d3,co);
  [wi,gi] =  sfun(nf,x0,y0,z0);
  e(i) = valder - gi*u;
end
toc

fprintf('emaxder = %5.2e RMSder = %5.2e\n', norm(e,inf),erms(e));
