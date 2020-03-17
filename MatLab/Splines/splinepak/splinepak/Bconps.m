%  Bconps 
% SplinePAK: Copyright Larry Schumaker 2014
% Computes a convex C^1 Powell-Sabin spline interpolating convex data

% Read in an initial triangulation
[no,xo,yo,nto,TRIo] = readtri;
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRIo);
figure; triplot(TRIo,xo,yo); 

% Sample a convex test function
z = zeros(no,1);
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
cdof = [z;grad]; 
co = A*cdof;

% Render the spline
ng = 51;
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check convexity
[dxmin,dymin,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng);
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dxmin,dymin,dermin);

% Set up some convexity constraints
m = input('input m for convexity constraints ');
C = conpatch(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,m);
Ac = -C*A;
sa = size(Ac); ncon = sa(1); nc = sa(2);
b = zeros(ncon,1);

% Set up the interpolation constraints
Aeq = sparse([eye(no),zeros(no,nc-no)]); beq = z;

% Solve the constrained minimization problem
obj = @(x,cdof) sum((x-cdof).^2);
options=optimset('TolFun',1e-4,'Display','off');

% Note: cdof must be a column vector
cds = size(cdof);
if cds(1) == 1
  cdof = cdof';
end

%options = optimset('MaxFunEvals',10000);

tic
[dofs,error,exitflag,output] = fmincon(@(x) obj(x,cdof),...
    cdof,Ac,b,Aeq,beq,[],[],[],options);
fprintf('error %g and flag %g\n',error,exitflag);
toc

% Find the B-coefs of the convex spline
c = A*dofs;

% Evaluate the convex spline on the grid
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the convex spline
figure; surfl(xg,yg,g');  colormap(copper);

% Compute the max and RMS errors
e = errg(xg,yg,g,f);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check the convexity
[dxmin,dymin,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng);
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dxmin,dymin,dermin);


% Check that final spline interpolates 
fprintf('check on interpolation: %5.2e\n ',norm(c(1:no) - z));
