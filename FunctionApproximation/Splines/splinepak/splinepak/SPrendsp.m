% SPrendsp
% SplinePAK: Copyright Larry Schumaker 2014
% Render a spherical spline

% Read a spherical triangulation
[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] =  slists(x,y,z,TRI);
ne = length(ie1);

d = input('input d ');
nc = n + ne*(d-1) + nt*(d-1)*(d-2)/2; 

%Set all coefficients to zero except for the m-th one
c = zeros(1,nc);  c(1) = 1;

% Scale the coeffients
scale = 4; c = c/scale;

% Evaluate the spline on sptri6
tic
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');
toc

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
axis vis3d; axis equal tight off; rotate3d on;
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
