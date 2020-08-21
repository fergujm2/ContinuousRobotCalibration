% Blsq15
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Fit scattered data by least squares using the space S^{12}_5
%    Uses the M-basis and solves the normal equations

% Read in a triangulation
[n,x,y,nt,TRI] = readtri; triplot(TRI,x,y);

% Compute the triangulation information
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the dimension of the spline space
dim = 3*n + ne;

% Input data points
[nd,xd,yd] = readxy;

% Sample Franke's function at the data points
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
[A,dof] = mds15(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,eadj,tstart,tadj,bdy);
 fprintf('time to set up the MDS %g\n',cputime - t1);

d = 5;
% Compute the B-coefficients of the least-squares spline
 [c,G,t1,t2] = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);
 fprintf('time to assemble and solve %g + %g\n',t1,t2);

% WARNING:  computing the condition number is slow for very large n
 fprintf('cond normal eqn  %5.2e\n',condest(G));

% Check C^1 smoothness
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
