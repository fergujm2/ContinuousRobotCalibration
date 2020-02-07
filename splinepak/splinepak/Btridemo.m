% Btridemo
% SplinePAK: Copyright Larry Schumaker 2014
% Read a triangulation from a data file and plot it
% Then create trianglulation data lists

% Read a triangulation
[n,x,y,nt,TRI] = readtri;

% Plot it
figure; triplot(TRI,x,y);

% Create the lists
tic
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);
toc

% Find the star of a vertex
k  = input('Find the star of vertex number k = ');
[nvstar,list] = starv(k,v1,v2,v3)

% Find the star of a triangle
k  = input('Find the star of triangle number k = ');
[ntstar,list] = startri(k,v1,v2,v3,tstart,tadj)

