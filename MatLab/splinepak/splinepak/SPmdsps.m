% SPmdsps
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a minimal determining set for the C1 Powell-Sabin spherical
%   spline space and plot a given M-basis function in the space

% Read in a spherical triangulation
[no,xo,yo,zo,nto,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] =  slists(xo,yo,zo,TRIo);
neo = length(ie1o);

% Find the Powell-Sabin refinement and the transformation matrix
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A,dof]...
= smdsps(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);

% Plot the refined triangulation
srendtri(x,y,z,ie1,ie2);

% Set one degree of freedom to one and the rest to zero
ndof = length(dof); df = zeros(ndof,1);
m = input('input number of basis function to plot '); df(m) = 1;

% Calculate all B-coefs
c = A*df;

% Check C1 smoothness
d = 2; sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Scale the coefficients for a better plot
c = c/(2*norm(c,inf));

% Evaluate the basis spline  on sptri6
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
axis vis3d; axis equal tight off; rotate3d on;
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
