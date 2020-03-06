% SPintsp  10/15/14
% SPAK: Copyright Larry Schumaker 2012
% Interpolates scattered data on the sphere with a C^0 linear spherical spline

%f = @(x,y,z)  x.^4 + 1.1*y.^4 + 1.3*z.^4;
f = @(x,y,z)  x.^2 + y.^2 + z.^2;

d = 1;
% Read the data and a triangulation
[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] =  slists(x,y,z,TRI);

% Sample the test function at the data points
c = f(x,y,z);

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

% Compute the error
err = g - sfun(nf,xp,yp,zp);
fprintf('emax = %5.2e RMS = %5.2e\n', norm(err,inf),erms(err));

% Read in gauss data
[wq,rq,sq,tq] = quadset(25);

nq = 25; 
m = input('input parameter for refinement m = ');
%m = round(sqrt(10000/length(v1)));
tic
 int = sphintsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,m,nq);
 intf = sphintf(x,y,z,v1,v2,v3,e1,e2,e3,ie1,f,m,nq);
toc

fprintf('integral of s = %10.8e, integral of f = %10.8e, \n',int,intf);
fprintf('difference = %10.8e, \n',abs(int-intf));
