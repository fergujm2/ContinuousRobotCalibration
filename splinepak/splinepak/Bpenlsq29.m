% Blpens29
% SplinePAK: Copyright Larry Schumaker 2014
% Solve the penalized least squares problem using the space S^{24}_9

% Read in a triangulation
[n,x,y,nt,TRI] = readtri;  %triplot(TRI,x,y);

% Compute the triangulation information
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the dimension of the spline space
dim = 3*n + ne;

% Input data points
[nd,xd,yd] = readxy;

% for random points
%   nd = input('input number of data points ';
% [xd,yd] = randxy(nd);

% Sample a function at the data points
f = @(x,y) franke2(x,y);
zd = f(xd,yd); wd = ones(nd,1);

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

t1 = cputime;
% Compute the transformation matrix
[A,dof] = mds29(x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,tstart,tadj);
fprintf('time to find MDS and A %g \n',cputime-t1);

% Compute the B-coefficients of the penalized least-squares spline
d = 9;
lambda = input('input lambda ');
[c,M,t1,t2] = penlsqbiv(d,x,y,v1,v2,v3,...
   e1,e2,e3,ie1,A,xd,yd,zd,wd,lambda);
fprintf('time to assemble and solve %g + %g \n',t1,t2);
fprintf('cond system  %g\n',cond(full(M)));

% Check C1 & C2 smoothness
cksmooth(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

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
