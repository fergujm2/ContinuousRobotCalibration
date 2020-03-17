% Brefine
% SplinePAK: Copyright Larry Schumaker 2014
% Uniformly refine a given triangulation

% Read and plot a triangulation
[no,xo,yo,nto,TRIo] = readtri;
figure; triplot(TRIo,xo,yo); axis off; axis equal;

% Refine the triangulation
tic
[x,y,TRI] = refine(xo,yo,TRIo);
toc

% Plot the refined triangulation
figure; triplot(TRI,x,y); 
n = length(x); nt = length(TRI); axis off; axis equal;
