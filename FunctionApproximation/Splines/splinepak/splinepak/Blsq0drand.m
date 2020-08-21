%  Blsq0drand
% SplinePAK: Copyright Larry Schumaker 2014
%  Compute the least-squares fit of samples of a function 
%   using a C0  spline of degree d 
%  Variant: uses random data points

% Read in a triangulation
[n,x,y,nt,TRI] = readtri;
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Input random data points and sample a function 
nd = input('input nd '); zd = zeros(nd,1);
[xd,yd] = randxy(nd);
f = @(x,y) franke2(x,y);
zd = franke2(xd,yd);
wd = ones(nd,1);

% Plot the triangulation and sample points
figure; hold; axis equal; axis off;
h = triplot(TRI,x,y); set(h,'LineWidth',1.5);
plot(xd,yd,'LineStyle','none','LineWidth',2,'Marker','o',...
  'MarkerFaceColor','g', 'MarkerEdgeColor','k',  'MarkerSize',5)

% Compute the coefficients of the lsq spline fit from S^0_d
to = cputime;
d = input('input d '); 
[c,G] = lsq0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,nd,xd,yd,zd,wd);
fprintf('computational time is %g \n ',cputime-to);
fprintf('condition number of G is %g \n ',cond(full(G)));

% Render the spline
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n', norm(e,inf),erms(e));

% Plot the x-derivative
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
