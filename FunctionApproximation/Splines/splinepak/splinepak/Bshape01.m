%  Bshape01  
% SPAK: Copyright Larry Schumaker 2012
% PURPOSE: Interpolate data at the vertices of a triangulation
%    with a C^0 linear spline

% Read in and plot the triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the values of the function at the vertices
z = zeros(n,1); 
for i = 1:n
  b = hill(x(i),y(i));
  z(i) = b(1);  
end

% Render the surface
d = 1; c = z; ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y); 
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error 
e = zeros(ng,ng);
for i = 1:ng
  xgi = xg(i);
  for j = 1:ng
    b = hill(xgi,yg(j));
    e(i,j) = b(1) - g(i,j);
  end
end
e = reshape(e,ng*ng,1);
fprintf('Maximum error: %e,   RMS = %e\n', norm(e,inf),erms(e))
