% SP15mds 
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a minimal determining set for the spherical Argyris spline 
%    space S^{12}_5 and plot a given M-basis function in the space

% Read in a spherical triangulation
[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] = slists(x,y,z,TRI);

% Compute the degrees of freedom and transformation matrix
[A,dof] = smds15(x,y,z,v1,v2,v3,e1,e2,e3,ie1);

% Set the degrees of freedom and compute the corresponding B-coefs
ndof = length(dof); df = zeros(ndof,1);
m = input('input basis function to plot ');
df(m) = 1; c = A*df; 

% Scale the coefficients for a better plot
c = c/(2*norm(c,inf));

% Check the C1 smoothness
d = 5; 
sc1ck(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the basis spline on sptri6
[G,gx,gy,gz] = rendsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri6');

% Plot the spline
figure; h = trisurf(G,gx,gy,gz);
axis vis3d; axis equal tight off; rotate3d on;
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
