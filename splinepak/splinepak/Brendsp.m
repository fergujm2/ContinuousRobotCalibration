% Brendsp
% SplinePAK: Copyright Larry Schumaker 2014
% Render a spline on a general polygonal domain

% Read in a triangulation
[nv,x,y,nt,TRI] = readtri; triplot(TRI,x,y);

% Compute the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Choose a degree and compute the number of coefs of a spline
d = input('input the degree of the spline d = ');
nc = nv + (d-1)*ne + choose(d-1,2)*nt

% Set one coef to one and the rest to zero
c = ones(nc,1);  c(6) = 2;

% Evaluate the spline on domain points of order m and
%   create a collection of triangular patches
m = input('input refinement level for the plot m = ');
[gx,gy,gz,gTRI] = rendspDP(d,m,x,y,v1,v2,v3,e1,e2,e3,ie1,c);

% Plot the spline 
figure; h = trisurf(gTRI,gx,gy,gz);
set(h,'FaceColor',[0 1 1])
