% Blsqps
% SplinePAK: Copyright Larry Schumaker 2014
%  Fit scattered data by least-squares using a C1 Powell-Sabin spline

% Read in a triangulation
[no,xo,yo,nto,TRIo] = readtri;   figure; triplot(TRIo,xo,yo);

% Compute the triangulation information
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRIo);

% Input data points
[nd,xd,yd] = readxy;

% Sample a test function at the data points and set weights
f = @(x,y) franke2(x,y);
zd = f(xd,yd);  wd = ones(nd,1);

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Find a minimal determing set and transformation matrix
tic
%[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
%  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro)
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,...
  dof,A] =  mdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,...
  ie1o,ie2o,trilo,triro,adjstarto,eadjo,bdyo);
toc

% Plot the refined triangulation
TRI = [v1,v2,v3]; figure; triplot(TRI,x,y);

% Compute the B-coefficients of the least-squares spline
d = 2;
[c,G,t1,t2] = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);
 fprintf('time to assemble and solve %g + %g \n',t1,t2);

% WARNING: computing the condition numbers can be slow for large problems
 fprintf('cond normal eqn  %g\n',cond(full(G)));

% Check C1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Render the spline
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the x-derivative
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
