% Bpens15g
% SplinePAK: Copyright Larry Schumaker 2014
% Fit noisy scattered data using a  penalized least squares spline
%   from the space S^{12}_5
% Computes PLSQ fits for a sequence of lambdas in [0,.006] and
% creates a graph of the RMS error versus lambda

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
zd = franke2(xd,yd); wd = ones(nd,1);

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Compute the transformation matrix
[A,dof] = mds15(x,y,v1,v2,v3,e1,e2,e3,...
  ie1,ie2,tril,trir,adjstart,eadj,tstart,tadj,bdy);

d = 5;  np = 21; ng = 51; lam = linspace(0,.006,np);
xg = linspace(0,1,ng); yg = xg; rms = zeros(np,1);
[X,Y] = meshgrid(xg,yg); fg = franke2(X,Y);
xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);

for i = 1:np
 [c,M,t1,t2] = penlsqbiv(d,x,y,v1,v2,v3,...
   e1,e2,e3,ie1,A,xd,yd,zd,wd,lam(i));
  fprintf('time to assemble and solve %g + %g \n',t1,t2);
%  fprintf('cond system  %g\n',cond(full(M)));
  [xg,yg,g] = valspgrid(d,x,y,v1,v2,v3, e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
   rms(i) = sqrt(sum(sum((fg-g').^2)))/ng;
end

% Plot a cubic spline fit to the RMS errors
[yp,cp] = notaknot(3,lam,rms);
 zp = sval2(3,yp,cp,lam);
figure; plot(lam,zp);
