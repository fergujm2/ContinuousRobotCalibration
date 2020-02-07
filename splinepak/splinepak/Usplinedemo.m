% Usplinedemo
% SplinePAK: Copyright Larry Schumaker 2014
% Plot a univariate spline and its 1st & 2nd derivatives
%   and compute definite integrals

% Set degree and number of interior knots
d = 3; k = 4; n = k + d + 1;
knots = [.1,.3,.5,.6]

% Find the extended knot sequence
y = [zeros(1,d+1),knots,ones(1,d+1)];

% Set the coefficients
c = [1,2,1,-1,1,3,-2,1];

% Compute the first and 2nd derivatives
[yd,cd] = derspl(d,y,c);
[yd2,cd2] = derspl(d-1,yd,cd);

% Set up the evaluation points
ng = 100; xg = linspace(y(1),y(n+1),ng);

% Evaluate the spline and its derivatives
zg = sval2(d,y,c,xg);
zg1 = sval2(d-1,yd,cd,xg);
zg2 = sval2(d-2,yd2,cd2,xg);

% Plot the spline and its 1st and 2nd derivative
figure; plot(xg,zg);
figure; plot(xg,zg1);
figure; plot(xg,zg2);

% Get coefficients of antiderivative spline
[cw,yw] = intsplco(d,y,c);

% Compute definite integrals
val1 = intspl(d,yw,cw,0,.5);
val2 = intspl(d,yw,cw,.5,1);
val3 = intspl(d,yw,cw,0,1);
fprintf('int [0,.5] = %g, [.5,1] = %g, [0,1] = %g \n',val1,val2,val3);

% Check on sum of integrals
val1 + val2
