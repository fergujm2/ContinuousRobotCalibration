% Bmds15 
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a minimal determing set dof for the Argyris macro-element space
%   and find the transformation matrix A. Plots individual basis functions

% Input a triangulation
[n,x,y,nt,TRI] = readtri;
triplot(TRI,x,y);

% compute the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
  vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Calculate the degrees of freedom and transformation matrix
[A,dof] = mds15(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,eadj,tstart,tadj,bdy);

% Set all coefficients to zero except for one
ndof = 6*n + ne; cdof = zeros(ndof,1);
k = input('choose dof to set to one ');
cdof(k) = 1;

% Compute the remaining B-coeffs
c = A*cdof; d = 5;

% Check the C1 smoothness of the spline
d = 5;
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Plot the chosen basis function

% Evaluate the chosen basis function on a grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[gx,gy,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(gx,gy,g'); colormap(copper);

