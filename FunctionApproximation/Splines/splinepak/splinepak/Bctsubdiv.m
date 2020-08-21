% Bctsubdiv
% SplinePAK: Copyright Larry Schumaker 2014
% Test the use of ctsubdiv to rewrite a cubic spline as a spline in 
%    the cubic Clough-Tocher space

% Read in a triangulation
[nv,x,y,nt,TRI] = readtri; triplot(TRI,x,y);

% Compute the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

d = 3;
%Compute the number of coefs of a spline
nc = nv + (d-1)*ne + choose(d-1,2)*nt

% Set the coefficients to random numbers
c = rand(nc,1);  

% Evaluate the spline on a rectangular grid
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[gx,gy,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,...
  ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(gx,gy,g'); colormap(copper);

% Subdivide the spline
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,cw] = ...
  ctsubdiv(x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);
triplot([v1,v2,v3],x,y);

% Evaluate the spline on a rectangular grid
[gx,gy,gw] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,cw,...
  ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(gx,gy,gw'); colormap(copper);

% Compute the maximum difference in the values of the splines at the grid
%   points in the triangulation
maxdiff = 0;
for i = 1:ng
  for  j = 1:ng
    if isnan(g(i,j)) < 1
      if abs(g(i,j) - gw(i,j)) > maxdiff      
        maxdiff = abs(g(i,j) - gw(i,j));
      end
    end
 end
end

fprintf('Max difference on a 51 x 51 grid = %5.2e \n',maxdiff);

