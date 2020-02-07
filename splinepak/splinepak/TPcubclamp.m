% TPcubclamp
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate data on a grid with a clamped bicubic spline

% Set up the grid
nx = input('input nx '); ny = input('input ny ');
xmin = 0; xmax = 1; ymin = 0; ymax = 1;
tx = linspace(xmin,xmax,nx); ty = linspace(ymin,ymax,ny);

% Find the spline space dimensions
n = nx+2; nw = ny+2;

% Sample the test function franke2d to get the data values
z = zeros(n,nw);

% Compute the values at the corners
der = franke2d(tx(1),ty(1)); 
z(1,1) = der(1); z(1,2) = der(3);
z(2,1) = der(2); z(2,2) = der(5);

der = franke2d(tx(1),ty(ny)); 
z(1,nw-1) = der(1); z(1,nw) = der(3);
z(2,nw-1) = der(2); z(2,nw) = der(5);

der = franke2d(tx(nx),ty(1)); 
z(n-1,1) = der(1); z(n-1,2) = der(3);
z(n,1) = der(2); z(n,2) = der(5);

der = franke2d(tx(nx),ty(ny)); 
z(n-1,nw-1) = der(1); z(n-1,nw) = der(3);
z(n,nw-1) = der(2); z(n,nw) = der(5);

% Values on the rest of first row
for j = 2:ny-1
 z(1,j+1) = franke2(tx(1),ty(j));
end

% Values on the rest of second row
for j = 2:ny-1
 der = franke2d(tx(1),ty(j)); 
 z(2,j+1) = der(2);
end

% Values for rows 3 through n-2
for i = 2:nx-1
 for j = 2:ny
  z(i+1,j+1) = franke2(tx(i),ty(j));
 end
 z(i+1,1) = franke2(tx(i),ty(1));
 der = franke2d(tx(i),ty(1));  z(i+1,2) = der(3);
 der = franke2d(tx(i),ty(ny));  z(i+1,nw) = der(3);
end

% Values for rest of row n-1
for j = 2:ny-1
 z(n-1,j+1) = franke2(tx(nx),ty(j));
end

% Values for rest of row n
for j = 2:ny-1
 der = franke2d(tx(nx),ty(j)); 
 z(n,j+1) = der(2);
end

% Compute the extended knot sequences and coef matrix
[xe,ye,c] = cubclamptp(tx,ty,z);

% Evaluate on a grid 
ng = 51; ngw = 51; d = 3; dw = 3; nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,xmin,xmax,ymin,ymax);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,@franke2);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline 
figure; surfl(xg,yg,g'); colormap(copper);
