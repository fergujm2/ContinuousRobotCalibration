%  SPpenlsg
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data on the sphere using a 
%   penalized least-squares Powell-Sabin spline or Clough-Tocher spline
% Uses a selection of lambdas, and plots the RMS error

% Input a triangulation
[no,xo,yo,zo,ntro,TRIo] = sreadtri;
[v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro] =  slists(xo,yo,zo,TRIo);

% Input data points
[nd,xd,yd,zd] = sreadpts;

% Choose a method and compute the transformation matrix
d = 2;
[x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A,dof]...
  = smdsps(xo,yo,zo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);

% sample a test function at the data points
nf = 4;
wd = sfun(nf,xd,yd,zd);
wt = ones(no,1); % set weights

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);  wd = wd + eps*rv;
end

% Choose lambdas to be used
nl = 25; lamg = linspace(0,.1,nl)'; 
maxg = zeros(nl,1); rmsg = zeros(nl,1);

lam = 0;
[Ml,Me,rhs] = spenlsqM(d,x,y,z,xd,yd,zd,wd,wt,v1,v2,v3,e1,e2,e3,ie1,A,lam);

% Loop on lambdas
for l = 1:nl
  M = Ml + lamg(l)*Me;
  dof = M \ rhs;
  c = A*dof;

% Evaluate the spline on sptri7
 [xp,yp,zp,g] = valsphsp(d,x,y,z,v1,v2,v3,e1,e2,e3,ie1,c,'sptri7');

% Compute the error
  err = g - sfun(nf,xp,yp,zp); 
  fprintf('lam = %8.5e, RMS = %5.2e\n',lamg(l),erms(err));
  maxg(l) = norm(err,inf); rmsg(l) = erms(err);
end

% Plot a cubic spline fit to the rms errors -- dropping the firstl
[yl,cl] = notaknot(3,lamg',rmsg);
sg = sval2(3,yl,cl,lamg);
figure; plot(lamg,sg);
