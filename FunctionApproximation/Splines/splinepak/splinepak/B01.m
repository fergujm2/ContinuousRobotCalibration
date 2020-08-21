%  B01 
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate data at the vertices of a triangulation
%  with a C^0 linear spline

% Read in and plot the triangulation
[n,x,y,nt,TRI] = readtri;
figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the values of the function at the vertices
%f = @(x,y) x + y;
f = @(x,y) franke2(x,y);

% Set the coefficients to the sample values
c = f(x,y);

% Evaluate the spline on a grid
ng = 51; d = 1;
xmin = min(x); xmax = max(x); ymin = min(x); ymax = max(x);
tic
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
toc

% Plot the spline
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));

return

% Alternate plot 
m = input('input m for resolution level = ');
tic
[gx,gy,gz,gTRI] = rendspDP(d,m,x,y,v1,v2,v3,e1,e2,e3,ie1,c);
toc

figure; h = trisurf(gTRI,gx,gy,gz);
set(h,'FaceColor',[0 1 1])
