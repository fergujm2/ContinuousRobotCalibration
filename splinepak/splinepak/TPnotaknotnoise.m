% TPnotaknotnoise
% SplinePAK: Copyright Larry Schumaker 2014
% Computes tensor product not-a-knot spline of odd degrees d,dw
%   to noisy data on a grid

% Input the degrees (note must be odd)
d = input('input x-degree (Odd) d =   '); 
dw = input('input y-degree (Odd) dw =   ');

% Set up the grid
a = 0; b = 1; aw = 0; bw = 1;
n = 201; nw = 201;
tx = linspace(a,b,n); ty = linspace(aw,bw,nw);

% Evaluate f on the grid
z = zeros(n,nw);
for i = 1:n
  z(i,:) = sigmoid(tx(i),ty);
end

% Add noise
eps = input('input eps ');
noise = readnoise(n*nw);
z = z +  eps*reshape(noise,n,nw);

% Compute the extended knot sequences and coef matrix
[xe,ye,c] = notaknottp(d,dw,tx,ty,z);

% Evaluate on a grid and compute the max and RMS errors
ng = 51; ngw = 51;  nu = 0; mu = 0;
[xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,a,b,aw,bw);

% Plot the spline 
figure; surfl(xg,yg,g'); colormap(copper);

% Compute the max and RMS errors on this grid
e = errg(xg,yg,g,@sigmoid);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
