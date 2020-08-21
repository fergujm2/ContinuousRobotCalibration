% TPcubquasi
% SPAK: Copyright Larry Schumaker 2012
% Fit data a grid with a bicubic quasi-interpolating spline

% Set up the grid
a = 0; b = 1; aw = 0; bw = 1;
nx = input('input nx '); ny = input('input ny ');
tx = linspace(a,b,nx); ty = linspace(aw,bw,ny);

% Find the spline space dimensions
n = nx+2; nw = ny+2;

% Sample franke2 to get the data values
z = zeros(nx,ny);
for i = 1:nx
   z(i,:) = franke2(tx(i),ty);
end

% Compute the extended knot sequences and coef matrix
[xe,ye,c] = cubquasitp(tx,ty,z);

% Evaluate on a grid 
ng = 51; ngw = 51;  d = 3; dw = 3; nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,a,b,aw,bw);

% Plot the spline 
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,@franke2);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
