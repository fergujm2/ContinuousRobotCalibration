% SPpsder
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data on the sphere with a Powell-Sabin spline
% Version of SPps that checks errors for derivatives

d = 2;
% Input a triangulation
[no,xo,yo,zo,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] = slists(xo,yo,zo,TRIo);

% Calculate frame vectors
[u,uw] = sframe(xo,yo,zo);

% Compute function values and gradients from a test function
nf = input('input nf ');
[w,grad] = sfun(nf,xo,yo,zo);

% Compute frame derivatives from gradients
for i = 1:no
  w1(i) = u(i,:)*grad(i,:)';
  w2(i) = uw(i,:)*grad(i,:)';
end

% Compute the coefs of the Powell-Sabin spline
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
  sphps(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,...
   u,uw,w,w1,w2);
toc

% Check C1 smoothness
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on sptri7 for error computations
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');

% Compute derivative errors at the data points in random directions
n = length(x); ne = length(ie1); nt = length(v1); np = length(xp);
fid = fopen('randpts.dat');
rp = fscanf(fid,'%g',[np,3]); rp = .25*rp;

[it,b1,b2,b3] = sfindtri(x,y,z,v1,v2,v3,xp,yp,zp);

e = zeros(1,np);
tic
for i = 1:np
  k = it(i);   x0 = xp(i); y0 = yp(i); z0 = zp(i);
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
