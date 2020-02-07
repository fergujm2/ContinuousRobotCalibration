% Bdelaunay
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Read data points from a file and create a Delaunay triangulation

% Read in points
[n,vx,vy] = readxy;

% Create the delaunay triangulation
tic
TRI = delaunay(vx,vy);
toc

% Plot the triangulation
figure; triplot(TRI,vx,vy);
