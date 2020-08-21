%  Bpenlsq29g
% SplinePAK: Copyright Larry Schumaker 2014
% Solve the penalized least squares problem using the space S^{24}_9
%  Computes the solution for a sequence of lambda in [0,.01]
%  and plots the resulting RMS errors

% Read in a triangulation
[n,x,y,nt,TRI] = readtri;  %triplot(TRI,x,y);

% Compute the triangulation information
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Input data points
[nd,xd,yd] = readxy;

% for random points
%   nd = input('input number of data points ';
% [xd,yd] = randxy(nd);


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
[A,dof] = mds29(x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,tstart,tadj);
fprintf('time to find MDS and A %g \n',cputime-t1);

% compute errors on a sequence of lambdas
d = 9;  np = 21; ng = 51; lam = linspace(0,.01,np);
xg = linspace(0,1,ng); yg = xg; rmserr = zeros(np,1);
[X,Y] = meshgrid(xg,yg); fg = franke2(X,Y);
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);

for i = 1:np
  [c,M,t1,t2] = penlsqbiv(d,x,y,v1,v2,v3,...
      e1,e2,e3,ie1,A,xd,yd,zd,wd,lam(i));
  fprintf('time to assemble and solve %g + %g \n',t1,t2);
%  fprintf('cond system  %g\n',cond(full(M)));
  [xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
   rmserr(i) = sqrt(sum(sum((fg-g').^2)))/ng;
end

% plot a cubic spline fit to the rms errors
[yp,cp] = notaknot(3,lam,rmserr);
 zp = sval2(3,yp,cp,lam);
figure; plot(lam,zp);
