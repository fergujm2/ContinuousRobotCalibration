% SPdelaunay
% SplinePAK: Copyright Larry Schumaker 2014
% Computes a spherical Delaunay triangulation for given points on the sphere

% Read points on the sphere
[n,x,y,z] = sreadpts;

% Construct the Delaunay triangulation
[v1,v2,v3] = sdelaunay(x,y,z);

% Get the edge lists
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] =  slists(x,y,z,[v1,v2,v3]);

% Plot the triangulation
srendtri(x,y,z,ie1,ie2);

