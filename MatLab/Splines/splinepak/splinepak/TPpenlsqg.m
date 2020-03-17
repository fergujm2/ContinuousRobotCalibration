% TPpenlsqg
% PURPOSE: Compute penalized least squares fits from noisy data 
%   on a grid using tensor-product splines for a sequence of lambda
%   Interpolate the RMS errors and plot the error curve

a = 0; b = 1; aw = 0; bw = 1;

% Define the sigmoidal test function
f = @(x,y) sigmoid(x,y);

% Set up the sample grid
a = 0; b = 1; aw = 0; bw = 1;
nd = 201; ndw = 201;
t = linspace(a,b,nd); tw = linspace(aw,bw,ndw);

% Evaluate f at the grid points
z = zeros(nd,ndw);
for i = 1:nd
 z(i,:) = f(t(i),tw);
end

% Add noise
eps = input('input eps ');
noise = readnoise(nd*ndw);
z = z +  eps*reshape(noise,nd,ndw);

% Input the degrees of the spline
d = input('input d '); dw = input('input dw ');

% Input the number of grid lines in each variable
m = input('input m '); mw = input('input mw ');

% Set up the extended knot sequences for  equally spaced knots
xe = [a*ones(1,d),linspace(a,b,m),b*ones(1,d)];
ye = [aw*ones(1,dw),linspace(aw,bw,mw),bw*ones(1,dw)];

ng = 51; ngw = 51; 
nl = 21; lam = linspace(0,.000005,nl);  rm = zeros(nl,1);

for i = 1:nl
  c = penlsqtp(d,dw,xe,ye,lam(i),t,tw,z);
  [xg,yg,g] =  valtpgrid(d,dw,xe,ye,c,nu,mu,ng,ngw,a,b,aw,bw);
  e = errg(xg,yg,g,f); rm(i) = erms(e); 
  fprintf('%5.2e %5.2e \n',lam(i),rm(i));
end

[yp,cp] = notaknot(3,lam,rm);
zp = sval2(3,yp,cp,lam);
figure; plot(lam,zp);


