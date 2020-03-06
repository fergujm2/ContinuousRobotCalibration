% SPps
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate Hermite data on the sphere with a  Powell-Sabin spherical spline 

d = 2;
% Read in a triangulation
[no,xo,yo,zo,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] = slists(xo,yo,zo,TRIo);

% Calculate the frame vectors
[u,uw] = sframe(xo,yo,zo);

% Compute function values and gradients at the vertices
nf = input('input nf ');
[w,grad] = sfun(nf,xo,yo,zo);

% Compute frame derivatives from gradients
for i = 1:no
  w1(i) = u(i,:)*grad(i,:)';
  w2(i) = uw(i,:)*grad(i,:)';
end

% Compute the coefs of the Powell-Sabin interpolant
tic
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
  sphps(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,...
   u,uw,w,w1,w2);
toc

% Plot the PS refined triangulation
srendtri2(no,nto,x,y,z,ie1o,ie2o,ie1,ie2);
% srendtri(x,y,z,ie1,ie2);

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

% Evaluate on sptri7 for error computation
tic
[xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');
toc

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));

return
%% Computation of the integral of the spline
nq = 25;
m = input('input parameter for quadrature refinement m = ');
%m = round(sqrt(10000/length(v1)));
tic
 integ = sphintsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,m,nq);
toc
fprintf('integral of s = %10.8e \n',integ);

return

%%% Alternate way of rendering a spherical spline

m = input('input parameter for rendering on domain pts m =  ');
tic
[gx,gy,gz,G] = srendspDP(d,m,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c);
toc
figure; h = trisurf(G,gx,gy,gz);
axis vis3d; axis equal tight off;  rotate3d on;
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
