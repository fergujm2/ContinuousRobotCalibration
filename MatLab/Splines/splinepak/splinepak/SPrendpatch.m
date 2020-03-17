% SPrendpatch  
% SplinePAK: Copyright Larry Schumaker 2014
% Render a spherical polynomial patch on one spherical triangle

% Read a triangulation
[n,x,y,z,nt,TRI] = sreadtri;

% Set the degree and B-coefficients of the patch
d = 3;  nc = (d+1)*(d+2)/2;
c = zeros(nc,1); c(8) = .5;

% Render the surface patch in red
m = input('input m ');
[gx,gy,gz,G] = valsphpol(d,m,x,y,z,c);
h2 = trisurf(G,gx,gy,gz);  
set(h2,'edgecolor',[0 0 0],'facecolor',[.8 .2 .2]);
axis vis3d; axis equal tight off; hidden on; rotate3d on; hold on;

% Render the spherical triangle in white
c = zeros(nc,1);
[gx,gy,gz,GT] = valsphpol(d,m,x,y,z,c);
h = trisurf(GT,gx,gy,gz); 
set(h,'edgecolor',[0 0 0],'facecolor',[1 1 1]);


