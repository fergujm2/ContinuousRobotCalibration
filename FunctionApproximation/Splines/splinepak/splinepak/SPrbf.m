% SPrbf
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data on the sphere with a radial basis function

% Define the radial basic function
rbf = @(e,r) exp(-(e*r).^2);  
eps = 3; %input('epsilon  ');

% Input data sites on the sphere
[n,x,y,z] = readxyz;
dsites = [x,y,z]; ctrs = dsites;

% Sample a test function at the data sites
nf = input('input nf '); w = zeros(n,1);
w = sfun(nf,x,y,z);

% Compute the coefs of the radial basis interpolant
eps = input ('input eps ');
tic
c = srbfinterp(dsites,w,eps,rbf);
toc

% Evaluate the RBF on the vertices of sptri6
tic
fid = fopen('sptri6');
np = fscanf (fid,'%d',1);
esites = fscanf(fid,'%g',[3,np]);  esites = esites';
xp = esites(:,1); yp = esites(:,2); zp = esites(:,3);
ntp = fscanf(fid,'%d',1);
G = fscanf(fid,'%d',[3,ntp]); G = G';
A = esites*ctrs';
EM = acos(A); EM = rbf(eps,real(EM));  
g = EM*c;
toc

% Plot the interpolant
gp = g + ones(np,1);
gx = gp.*xp; gy = gp.*yp; gz = gp.*zp;
figure; h = trisurf(G,gx,gy,gz);
set(h,'edgecolor',[0 0 0],'facecolor',[1 .8 .65]);
axis vis3d; axis equal tight off;  hidden on; rotate3d on;

% Compute the error on the grid
np = length(xp); e = zeros(np,1);
for i = 1:np
  wi =  sfun(nf,xp(i),yp(i),zp(i));
  e(i) = wi - g(i);
end

fprintf('emax = %5.2e RMS = %5.2e\n', norm(e,inf), erms(e));
