% TP01
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate data on a grid with a C^0 bilinear tensor product spline
%  works on a rectangle [a,b] x [aw,bw]

% Input the rectangle
a = 0; b = 1; aw = 0; bw = 1;

% Input the number of grid lines in each direction
n = input('input n '); nw = input('input nw ');

% Set up equally spaced grid lines
t = linspace(a,b,n); tw = linspace(aw,bw,nw);

% Evaluate a test function on the grid
z = zeros(n,nw);
for i=1:n
  z(i,:) = franke2(t(i),tw);
end

% Compute the extended knot sequences and the coef matrix
[xe,ye,c] = bilintp(t,tw,z);

% Plot the spline 
figure; surfl(t,tw,z'); colormap(copper);

% Evaluate on a grid
ng = 51; ngw = 51; d = 1; dw = 1; nu = 0; mu = 0;
[xg,yg,g] = valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,a,b,aw,bw);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,@franke2);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
