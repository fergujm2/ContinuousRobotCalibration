% Blsqwang  11/15/12
% SplinePAK: Copyright Larry Schumaker 2014
% Fit scattered data by least squares with the Wang C2 macro-element 
%   space S^{24}_9


% Read in a triangulation
[no,xo,yo,nto,TRI] = readtri;   figure; triplot(TRI,xo,yo);

% Compute the triangulation information
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);

% Compute the dimension of the spline space 
dim = 6*no + 3*neo;

% Input data points and sample a function at the data points
[nd,xd,yd] = readxy; 
f = @(x,y) franke2(x,y);
zd = f(xd,yd); wd = ones(nd,1);

% Compute the degrees of freedom and transformation matrix
t1 = cputime;
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,tadj,tstart, ...
   dof,A]  = mdswang(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o, ...
     ie1o,ie2o,trilo,triro,tstarto,tadjo);

% Plot the refined triangulation
figure; triplot([v1,v2,v3],x,y);

% Compute the B-coefficients of the least-squares spline
d = 5;
[c,G,t1,t2] = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);
%[c,G,t1,t2] = lsqbivnew(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd); % Faster
fprintf('time to assemble and solve %g + %g \n',t1,t2);
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
