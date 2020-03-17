% SPrefine 
% SplinePAK: Copyright Larry Schumaker 2014
% Creates a uniform refinement of a given spherical triangulation

% Input a spherical triangulation
[no,xo,yo,zo,nto,TRIo] = sreadtri;

% Create the edge lists
tic
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] =  slists(xo,yo,zo,TRIo);
toc

% Render the triangulation
srendtri(xo,yo,zo,ie1o,ie2o);

% Refine the triangulation
[x,y,z,v1,v2,v3] =  srefine(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o);

% Create the new edge lists
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] = ...
     slists(x,y,z,[v1,v2,v3]);

% Render the refined triangulation
srendtri(x,y,z,ie1,ie2);

