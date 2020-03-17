%  Bpos01  
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate a known nonnegative function at the vertices of a triangulation
%    with a nonnegative C0 linear spline

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

% Evaluate the spline on a grid
d = 1; c = z; ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y); 
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error 
e = errg(xg,yg,g,@hill);
fprintf('Error:   emax = %5.2e,   RMS = %5.2e\n', norm(e,inf),erms(e))

% Find the minimum of the function on the grid
fprintf('The minimum of the spline = %5.2e\n', min(min(g)));

return

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,gx] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,gx');  colormap(copper);
