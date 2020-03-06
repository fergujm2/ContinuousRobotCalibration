% TPnotaknot
% SplinePAK: Copyright Larry Schumaker 2014
% Computes tensor product not-a-knot spline of odd degrees d,dw
%   to data on a grid

% Input the degrees (note must be odd)
d = input('input degree in x-variable (Odd) d =   '); 
dw = input('input degree in y-variable (Odd) dw =   ');

% Set up the grid
a = 0; b = 1; aw = 0; bw = 1;
n = input('input number of x-grid lines n =   '); 
nw = input('input number of y-grid lines nw =   ');
tx = linspace(a,b,n); ty = linspace(aw,bw,nw);

% Evaluate franke2 on the grid
z = zeros(n,nw);
for i = 1:n
  z(i,:) = franke2(tx(i),ty);
end

% Compute the extended knot sequences and coef matrix
[xe,ye,c] = notaknottp(d,dw,tx,ty,z);

% Evaluate on a grid 
ng = 51; ngw = 51; nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,a,b,aw,bw);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,@franke2);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Plot the spline 
figure; surfl(xg,yg,g'); colormap(copper);
