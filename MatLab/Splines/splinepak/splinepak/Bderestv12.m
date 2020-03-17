% Bderestv12
% SplinePAK: Copyright Larry Schumaker 2014
% Estimate the x and y derivatives at (.5,.5) of Franke's function 
%   using 12 nearby points 
% Can do polynomial least-squares or RBF interpolation

% Fix the evaluation point
x = .5; y = .5; 

% Find the true x and y derivatives at (.5,.5);
 der = franke2d(x,y);  dx = der(2); dy = der(3);

% Set up 12 points in [0,1] x [0,1]
A = [0.6066    0.1688
     0.5374    0.2731
     0.0975    0.1211
     0.4952    0.0596
     0.1125    0.7512
     0.9405    0.3062
     0.7775    0.7186
     0.1555    0.6575
     0.7570    0.3239
     0.9873    0.0103
     0.2751    0.0493
     0.3981    0.6670];

xo = A(:,1); yo = A(:,2); nd = 12;

% Loop to test on a sequence of scaled points
for i = 1:5
  m = 2^(i-1);
% Scale the points and center around (x,y)
  xd = x + (xo-x)/m; yd = y + (yo-y)/m;  

% Sample the test function
  zd = franke2(xd,yd);

% Plot the points
  figure; hold; 
  plot(x,y,'LineStyle','none','LineWidth',2,'Marker','*',...
  'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',20)
  plot(xd,yd,'LineStyle','none','LineWidth',2,'Marker','o',...
  'MarkerFaceColor','g', 'MarkerEdgeColor','k',  'MarkerSize',10)
  axis equal; hold off;

% Estimate the derivatives using a least squares quadratic polynomial
   d = 2;
  [derx,dery,A] = derestlsqv(d,xd,yd,zd,x,y);
cond(A)

% uncomment for rbf test
%  eps = 3; [derx,dery,A] = derestrbfv(xd,yd,zd,x,y,eps)


% Compute errors
  ex = abs(dx-derx); ey = abs(dy-dery);
  fprintf('errx = %5.2e, erry = %5.2e, cond = %g \n',ex,ey,cond(A));
  fprintf('\n');
end
