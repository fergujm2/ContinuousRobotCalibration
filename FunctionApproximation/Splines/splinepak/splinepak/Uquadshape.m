% Uquadshape
% SPAK: Copyright Larry Schumaker 2012
% Interpolate data with a C^1 quadratic spline with shape control

% Set the nodes and values to be interpolated
m = 7;
t = [0 .2,.4,.5,.6 .7 1];
z = [.2 .2 0 0 .5 .8 1];

% Compute the extended knot sequence and coefficient vector
[y,c] = quadshape(t,z);

% Plot the data points
figure; hold; plot(t,z,'LineStyle','none','LineWidth',2,'Marker','o',...
                'MarkerFaceColor','g',...
                'MarkerEdgeColor','k',...
 'MarkerSize',10)

% Find the dimension of the spline space
n = 2*m;

% Plot the spline
ng = 100; d = 2;
xg = linspace(0,1,ng);
zg = sval2(d,y,c,xg);
plot(xg,zg)

