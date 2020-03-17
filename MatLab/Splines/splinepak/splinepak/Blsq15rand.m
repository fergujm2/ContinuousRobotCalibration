%  Blsq15rand
% SplinePAK: Copyright Larry Schumaker 2014
%  Fit scattered data by least squares using the space S^{12}_5
%  Variant: Uses random data points

% Read in a triangulation
[n,x,y,nt,TRI] = readtri; triplot(TRI,x,y);

% Compute the triangulation information
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Compute the dimension of the spline space
dim = 3*n + ne;

% Input data points
nd = input('input number of data points ');
[xd,yd] = randxy(nd);

% Sample a function at the data points
zd = franke2(xd,yd); wd = ones(nd,1);

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

% Compute the B-coefficients of the least-squares spline
d = 5;
 [c,G,t1,t2] = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);
 fprintf('time to assemble and solve %g + %g\n',t1,t2);
% WARNING:  computing the condition number is slow for n large
 fprintf('cond normal eqn  %g\n',condest(G));

% Check C^1 smoothness
c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Render the spline
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,@franke2);
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n', norm(e,inf),erms(e));

% Plot the x-derivative
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error of the x-derivative
eder = errgder(xg,yg,g,@franke2d,2);
fprintf('Derivative: emax =%5.2e, RMS = %5.2e\n',norm(eder,inf),erms(eder));
