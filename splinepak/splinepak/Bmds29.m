% Bmds29
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a minimal determing set dof for the 
% the macro-element space S^{24}_9 and find the transformation matrix A
% Plots individual basis functions

% Input a triangulation
[n,x,y,nt,TRI] = readtri;
%figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the degrees of freedom and transformation matrix
[A,dof] = mds29(x,y,v1,v2,v3, ...
   e1,e2,e3,ie1,ie2,tril,trir,bdy,tstart,tadj);

% Pick a degree of freedom to set to one
ndof = length(dof);
cdof = zeros(ndof,1);
icd = input(' input dof to set to one '); cdof(icd) = 1;

% Compute the remaining coefs of the spline
c = A*cdof;

% Check the C1 & C2 smoothness
d = 9;
cksmooth(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Render on a grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[gx,gy,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(gx,gy,g'); colormap(copper);

