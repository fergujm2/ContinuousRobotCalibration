% Ucubclampest
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE: Interpolate data with a cubic clamped spline

% Define the function to be interpolated
f = @(x) exp(x).*sin(2*pi*x);

% Define its derivative
fd = @(x) exp(x).*(sin(2*pi*x) + 2*pi*cos(2*pi*x));

% Input the number of sample points
m = input('input the number of sample points m =  ');    

% Choose equally sample points in [0,1] and sample f at these points
a = -1; b = 1; t = linspace(a,b,m); z = f(t);

% Estimate the derivative at a 
M = zeros(5);
tv = [t(1),t(2),t(3),t(4),t(5)];
for i = 1:5
  M(i,:) = tv.^(i-1);
end
c = M \ [0;1;2*a;3*a^2;4*a^3];
alpha = [z(1),z(2),z(3),z(4),z(5)]*c;

% Estimate the derivative at b 
M = zeros(5);
tv = [t(m-4),t(m-3),t(m-2),t(m-1),t(m)];
for i = 1:5
  M(i,:) = tv.^(i-1);
end
c = M \ [0;1;2*b;3*b^2;4*b^3];
beta = [z(m-4),z(m-3),z(m-2),z(m-1),z(m)]*c;

% Compute the extended knot sequence and coefficient vector
[y,c] = cubclamp(t,z,alpha,beta);

% Set dimension of the spline space
n = m+2;

% Evaluate the spline at ng equally spaced points
ng = 100;  d = 3;
xg = linspace(a,b,ng); 
zg = sval2(d,y,c,xg);

% Plot the spline
figure; plot(xg,zg);

% Compute the max and RMS errors
fg = f(xg); e = fg-zg;
fprintf('maxerr = %5.2e,  rms = %5.2e\n',norm(e,inf),erms(e));

