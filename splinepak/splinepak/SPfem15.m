% SPfem15
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Solve a PDE on the sphere using spherical spline space S^12_5

[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] = slists(x,y,z,TRI);

% Example 11.56
os = input('input omega^2 ');
f = @(x,y,z) (30 + os)*x.^2.*z.^3 - 2*z.^3 - 6*x.^2.*z;
true = @(x,y,z) x.^2.*z.^3;

% Example 11.57
%f = @(x,y,z) (os +x.^2 + 2*x-1).*exp(x);
%true = @(x,y,z) exp(x);

% Find the transformation matrix
tic
[A,dof] = smds15(x,y,z,v1,v2,v3,e1,e2,e3,ie1); d = 5;
toc
d = 5;

tic
[c,M] = sfem(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,A,os,f);
toc

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
