% SPrendf
% SPAK: Copyright Larry Schumaker 2012
% Render a spherical spline


nf = input('input nf ');
f = @(x,y,z) sfun(nf,x,y,z);

%f = @(x,y,z) exp(x);
%f = @(x,y,z) x.^2.*z.^3;

fname = 'sptri6';
[np,xp,yp,zp,nt,G] = sreadtri(fname);

fv = f(xp,yp,zp) + 1;
xp = xp.*fv;
yp = yp.*fv;
zp = zp.*fv;


% Plot the spline
figure; h = trisurf(G,xp,yp,zp);
axis vis3d; axis equal tight off; rotate3d on;
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
