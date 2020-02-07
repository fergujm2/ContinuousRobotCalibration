% Blsq29
% SplinePAK: Copyright Larry Schumaker 2014
%  Fit scattered data by least squares using the macro-element space S^{24}_9

% read in a triangulation
[n,x,y,nt,TRI] = readtri;
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Input data points and sample a function at the data points
[nd,xd,yd] = readxy;  
f = @(x,y) franke2(x,y);
zd = f(xd,yd); wd = ones(nd,1);

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Compute the dof and transformation matrix A
t1 = cputime;
[A,dof] = mds29(x,y,v1,v2,v3, ...
   e1,e2,e3,ie1,ie2,tril,trir,bdy,tstart,tadj);
fprintf('time to find MDS and A %g \n',cputime-t1);

% compute dimension of the spline space
d = 9; dim = 15*n + 3*ne + nt;

% Compute the B-coefficients of the least-squares spline
[c,G,t1,t2] = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);
fprintf('time to assemble and solve %g + %g \n',t1,t2);
%fprintf('cond normal eqn  %g\n',cond(full(G)));

% Check the C^1 & C^2 smoothness
cksmooth(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

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
