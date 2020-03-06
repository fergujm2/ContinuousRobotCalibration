% Bposlsq01
% SplinePAK: Copyright Larry Schumaker 2014
% Fit scattered data with a nonnegative C0 linear spline
%   using least-squares

% Read in a triangulation
[n,x,y,nt,TRI] = readtri;

% Read data and compute Delaunay triangulation
%[n,x,y] = readxy; TRI = delaunay(x,y); figure; triplot(TRI,x,y);

[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Read in random data points
nd = input('input nd ');
[xd,yd] = randpts(nd);

% Sample a test function
zd = hill(xd,yd); 

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Plot the data points on the triangulation
figure; hold; axis equal; axis off;
h = triplot(TRI,x,y); set(h,'LineWidth',2.5);
plot(xd,yd,'LineStyle','none','LineWidth',4,'Marker','.',...
  'MarkerFaceColor','g', 'MarkerEdgeColor','k',  'MarkerSize',20)

% Compute the B-coefficients of lsq spline fit from S^0_d
[c,G] = lsq01(x,y,v1,v2,v3,e1,e2,e3,ie1,xd,yd,zd);

% Evaluate the spline on a grid
ng = 51; d = 1; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error 
e = errg(xg,yg,g,@hill);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check the minmum value of the spline
fprintf('Minimum of the spline: %5.2e\n', min(min(g)))

% Make the spline nonnegative by adjusting coefficients to be nonnegative
c = max(c,0);

% Evaluate the adjusted spline on the grid
ng = 51; d = 1;
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the nonnegative spline
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error 
e = errg(xg,yg,g,@hill);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% check the minimum value of the spline
fprintf('Minimum of the spline: %5.2e\n', min(min(g)))
