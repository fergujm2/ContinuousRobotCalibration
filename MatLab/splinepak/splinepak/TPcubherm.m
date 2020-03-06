% TPcubherm
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate hermite data on a grid using
%    a  C^{1,1} bicubic tensor product spline

% Set up the grid
a = 0; b = 1; aw = 0; bw = 1;
nx = input('input number of x-grid lines nx = '); 
ny = input('input number of y-grid lines ny = ');
tx = linspace(a,b,nx); ty = linspace(aw,bw,ny);

% Find the spline space dimensions
n = 2*nx; nw = 2*ny;

% Sample franke2d to get the data values
z = zeros(n,nw);
nr = 0; nc = 0;
for i = 1:nx
 nr  = 2*(i-1)+1;
 for j = 1:ny
   nc = 2*(j-1)+1;
   der = franke2d(tx(i),ty(j));
   z(nr,nc) = der(1);
   z(nr+1,nc) = der(2);
   z(nr,nc+1) = der(3);
   z(nr+1,nc+1) = der(5); 
 end
end

% Compute the extended knot sequences and coef matrix
[xe,ye,c] = cubhermtp(tx,ty,z);

% Evaluate on a grid and compute the max and RMS errors
ng = 51; ngw = 51;  d = 3; dw = 3; nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu, ng,ngw,a,b,aw,bw);

% Plot the spline 
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,@franke2);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
