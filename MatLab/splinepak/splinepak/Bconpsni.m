% Bconpsni 
% SplinePAK: Copyright Larry Schumaker 2014
% Computes a convex C^1 Powell-Sabin spline nearly-interpolating on a convex
%  data set. Uses the minimal energy Powell-Sabin spline in the 1st stage

% Read in an initial triangulation
[no,xo,yo,nto,TRIo] = readtri;
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRIo);
figure; triplot(TRIo,xo,yo);

% Sample a convex test function
f = @(x,y) conf(x,y);
z = conf(xo,yo);

% Modify the triangulation by applying the swap algorithm
TRIs = conswap(v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,xo,yo,z);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRIs);
figure; triplot(TRIs,xo,yo);

% Compute the Powell-Sabin split, degrees of freedom, and
%    transformation matrix
to = cputime;
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
tm = cputime - to;
d = 2;

% Compute the minimal energy C^1 quadratic Powell-Sabin interpolant
[grad,M,t1,t2] = menpsn(v1o,v2o,v3o,x,y,z, ...
   v1,v2,v3,e1,e2,e3,ie1,A);
fprintf('Time: nmds, assemble and solve %g %g %g \n',tm,t1,t2);
fprintf('condition number %g \n',cond(full(M)));
cdof = [z;grad]; co = A*cdof;

% Set up the degrees of freedom and compute the B-coefficients
dof = [z;grad];  co = A*dof; 

% Evaluate the first-stage interpolating spline on a grid
ng = 51; d= 2;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the max and RMS errors of this spline on the grid
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check convexity
[dxmin,dymin,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng);
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dxmin,dymin,dermin);


% Set up convexity constraints
m = input('input m for the order of the convexity constraints ');
C = conpatch(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,m);
CA = C*A;
sa = size(CA); ncon = sa(1); nc = sa(2);
b = zeros(ncon,1);

% Solve the constrained minimization problem

% Note: dof must be a column vector
cds = size(dof);
if cds(1) == 1
  dof = dof';
end

% Choose starting DOF
%zxo = zeros(no,1); dofo = [ones(no,1);zxo;zxo];
dofo = zeros(3*no,1);

tic
obj = @(x,dof) sum((x-dof).^2);
options=optimset('TolFun',1e-3,'Display','off');

[dofs,error,exitflag,output] = fmincon(@(x) obj(x,dof),...
    dofo,-CA,b,[],[],[],[],[],options);
fprintf('error %g and flag %g\n',error,exitflag);
toc

% Find the B-coefs of the spline
c = A*dofs;

% Evaluate the new spline on a grid
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the errors
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check the convexity
[dxmin,dymin,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng);
fprintf('dx,dy,dermin %e, %e, %e\n',dxmin,dymin,dermin);

% Check how close it comes to interpolating
e = c(1:no) - z; md = max(abs(e)); rmsd = norm(e);
fprintf('max and RMS difference at interp pts = %5.2e, %5.2e\n',md,rmsd/no);

