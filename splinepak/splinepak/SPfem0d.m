% SPfem0d
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Solve a PDE on the sphere with  C^0 spherical splines of degree d

os = 100;
% 3rd degree
%f = @(x,y,z) (12 + os)*(y.^3 +z.^3) -6*(y+z);
%true = @(x,y,z) y.^3 +z.^3;

%f = @(x,y,z) (20 + os)*y.^4 - 12*y.^2;
%true = @(x,y,z) y.^4;

% 5th degree
f = @(x,y,z) (30 + os)*x.^2.*z.^3 - 2*z.^3 - 6*x.^2.*z;
true = @(x,y,z) x.^2.*z.^3;


f = @(x,y,z) (os + x.^2 + 2*x -1).*exp(x);
%f = @(x,y,z) (os -1).*exp(x);
true = @(x,y,z) exp(x);

%f1 = @(x,y,z) 1-sqrt(2-2.*z);
%f14 = @(x,y,z) (f1(x,y,z)>0).*f1(x,y,z).^4;
%f16 = @(x,y,z) (f1(x,y,z)>0).*f1(x,y,z).^6;
%f2 = @(x,y,z) (25*z.^2 -9*z + 4*z.*sqrt(2-2*z)-15);
%f2p = @(x,y,z) (f2(x,y,z)>0).*f2(x,y,z);
%f3 = @(x,y,z) 35*(2-2*z) + 18*sqrt(2-2*z)+3;
%f = @(x,y,z) 112*f14(x,y,z) .*f2(x,y,z) + os*f16(x,y,z).*f3(x,y,z);
%u = @(x,y,z) f1p(x,y,z).*f3(x,y,z);
%f = @(x,y,z) 112*f14(x,y,z) .*f2(x,y,z) + os*f16(x,y,z).*f3(x,y,z);
%u = @(x,y,z) f16(x,y,z).*f3(x,y,z);

[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] = slists(x,y,z,TRI);

d = input('input d ');  os =100;

[c,M] = sfem0d(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,os,f);

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
