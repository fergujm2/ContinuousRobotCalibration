% Bfindtri
% SplinePAK: Copyright Larry Schumaker 2014
% Constructs a Delaunay triangulation for n random points
% Also tests tsearchn for locating points in a triangulation

n = input('input n '); 
x = rand(n,1); y = rand(n,1);

% Create the Delaunay triangulation
TRI = delaunay(x,y);

% Create np random points
np = input('input np '); 
tx = rand(np,1); ty = rand(np,1);

% Find their locations and barycentric coordinates
tic
[tnum,b] = tsearchn([x,y],TRI,[tx,ty]);
toc
