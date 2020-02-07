% U01
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolates data using a C^0 linear spline

% Choose the sample points
t = [0 .25 .5 .75  1];

% Choose values to be interpolated 
z = [1 2 0 .75 -1];

% plot the spline
figure; h = plot(t,z,'LineWidth',2);


