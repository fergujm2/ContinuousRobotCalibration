%  Bmon01data 
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a monotone C^0 linear spline interpolating monotone data on a grid
% Version: Fixes the data

% Read a triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Data values
z(1) = 0; z(2) = 0; z(3) = 0; z(4) = 1; z(5) = 1;

% Evaluate the spline on a grid
d = 1; 
c = z;
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,gx] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,gx');  colormap(copper);
fprintf('The minimum of the $x$ derivatives = %5.2e\n', min(min(gx)));

% Evaluate the y-derivative on the grid
u = [0,1];
[xg,yg,gy] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);

% Plot the y-derivative
figure; surfl(xg,yg,gy');  colormap(copper);
fprintf('The minimum of the $y$ derivatives = %5.2e\n', min(min(gy)));
