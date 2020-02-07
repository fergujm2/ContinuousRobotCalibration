% Ushape
% SplinePAK: Copyright Larry Schumaker 2014
% Compare some interpolants for shape properties

% Set the nodes and values to be interpolated
m = 7;
t = [0 .2,.4,.5,.6 .7 1];
z = [.2 .2 0 0 .5 .8 1];

% Plot the data points
figure; hold; plot(t,z,'LineStyle','none','LineWidth',2,'Marker','o',...
                'MarkerFaceColor','g',...
                'MarkerEdgeColor','k',...
 'MarkerSize',10)

% plot the linear spline interpolant
plot(t,z);

% plot the not-a-knot interpolant
ng = 100; xg = linspace(0,1,ng); d = 3; 
[y,c] = notaknot(d,t,z);
zg = sval2(d,y,c,xg);
plot(xg,zg,'r');

% plot the cubic clamped interpolant with zero end derivatives
alpha = 0; beta = 0;
[y,c] = cubclamp(t,z,alpha,beta);
n = m+2; xg = linspace(0,1,ng);
zg = sval2(d,y,c,xg);
plot(xg,zg,'g');

% plot the cubic Hermite interpolant with zero derivatives
z1 = zeros(1,m);
[y,c] = cubherm(t,z,z1);
n = 2*m; zg = sval2(d,y,c,xg);
plot(xg,zg,'b');


