% Bmdsps
% SplinePAK: Copyright Larry Schumaker 2014
% Chooses a minimal determining set for S^1_2 and computes the 
%   associated transformation matrix A
% Plots individual M-basis functions

% Read a triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the degrees of freedom and the transformation matrix
tic
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,dof,A] = ...
  mdsps(x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,...
  tril,trir,adjstart,eadj,bdy);
toc

% Plot the refined triangulation
TRI = [v1,v2,v3]; figure; triplot(TRI,x,y);

% Pick a degree of freedom to set to one (and set the rest to zero)
ndof = length(dof);  cdof = zeros(ndof,1);
m = input('choose dof to set to one ');
cdof(m) = 1;

% Compute all B-coefficients
c = A*cdof; d = 2;

% Check for C1 continuity
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the basis function on a grid on the smallest enclosing rectangle
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the basis function
figure; surfl(xg,yg,g');  colormap(copper);

