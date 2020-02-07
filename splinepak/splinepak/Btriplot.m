% Btriplot
% SplinePAK: Copyright Larry Schumaker 2014
%  PURPOSE: Read a triangulation from a data file and plot it

% read the triangulation
[n,x,y,nt,TRI] = readtri;

% plot it
figure; triplot(TRI,x,y);
