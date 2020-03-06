% Bdelaunayrand
% SplinePAK: Copyright Larry Schumaker 2014
% inputs random data points and finds a Delaunay triangulation

n = input('input n ');
p = rand(n,2); x= p(:,1); y = p(:,2);

% Compute the delaunay triangulation
tic
TRI = delaunay(x,y);
toc

% Plot the triangulation
figure; triplot(TRI,x,y); axis off;

