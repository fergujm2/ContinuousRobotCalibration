% Bconlsqps 
% SplinePAK: Copyright Larry Schumaker 2014
% Computes a convex C^1 Powell-Sabin spline fitting noisy data
%   at scattered data points.  The first stage computes the least-squares
%   spline, and in the second stage we solve a quadratic programming
%   problem subject to convexity constraints 

% Input a triangulation
[no,xo,yo,nto,TRIo] = readtri;
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRIo);

% Input random data points and sample a convex function 
nd = input('input number of data points nd ');
[xd,yd] = randxy(nd);
zd = conf(xd,yd);
wd = ones(nd,1);

% Plot the triangulation and data points
figure; hold;
plot(xo,yo,'LineStyle','none','LineWidth',2,'Marker','*',...
  'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',20)
h = triplot(TRIo,xo,yo); set(h,'LineWidth',1.5);
plot(xd,yd,'LineStyle','none','LineWidth',2,'Marker','o',...
  'MarkerFaceColor','g', 'MarkerEdgeColor','k',  'MarkerSize',5)
hold off;

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Compute the Powell-Sabin split and transformation matrix A
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);

% Compute the LSQ fit
d = 2;
[co,G,t1,t2] = lsqbiv(d,x,y,v1,v2,v3,e1,e2,e3,ie1,A,xd,yd,zd,wd);

% Plot the refined triangulation
triplot([v1,v2,v3],x,y);

% Evaluate the spline fit on a grid
ng = 51;  xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the errors
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check convexity
[dxmin,dymin,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng);
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dxmin,dymin,dermin);

% Second Stage:   Set up convexity constraints
m = input('input m for the order of the convexity constraints ');
C = conpatch(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,m);
CA = C*A;
sa = size(CA); ncon = sa(1); nc = sa(2);
b = zeros(ncon,1);

% Get the DOF for the initial fit 
[zx,zy] = gradps(no,nto,x,y,ie1o,ie2o,eadjo,adjstarto,co);
dof = [co(1:no);zx;zy];

% Solve the constrained minimization problem
obj = @(x,dof) sum((x-dof).^2);
d0 =  zeros(3*no,1);
options=optimset('TolFun',1e-4,'Display','off');
tic
[dofs,error,exitflag,output] = fmincon(@(x) obj(x,dof),...
    d0,-CA,b,[],[],[],[],[],options);
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
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dxmin,dymin,dermin);
