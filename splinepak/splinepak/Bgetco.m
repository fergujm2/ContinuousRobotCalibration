% Bgetco
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Test the functions getco and getindex

% Read a triangulation and plot it
[nv,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);

% Create the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Choose the degree of spline and set the coefs
d = input('input the degree of the spline d = ');
nc = nv + (d-1)*ne + (d-1)*(d-2)*nt/2; c = [1:nc];

% Choose a triangle and find the associated coefs and indices
k = input('input the triangle number ');
co = getco(d,k,nv,v1,v2,v3,e1,e2,e3,ie1,c)
ic = getindex(d,k,nv,v1,v2,v3,e1,e2,e3,ie1)


