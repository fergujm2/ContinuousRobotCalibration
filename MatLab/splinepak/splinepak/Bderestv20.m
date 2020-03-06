% Bderestvs20
% SplinePAK: Copyright Larry Schumaker 2014
% Estimate the x and y derivatives at (.5,.5) of Franke's function 
%   using 20 nearby points. Here use least squares with a cubic polynomial

% Fix the evaluation point
x = .5; y = .5; 

% Compute the true x and y derivatives at (.5,.5)
 der = franke2d(x,y);  dx = der(2); dy = der(3);

% Fix the 20 points in [0,1] x [0,1]
A = [0.502292 0.0471659
0.209907 0.678769
0.528482 0.724696
0.801893 0.0248922
0.837946 0.606643
0.744085 0.202125
0.216183 0.0453878
0.363933 0.965868
0.480172 0.09101
0.360622 0.561628
0.574392 0.936005
0.150454 0.512006
0.082784 0.324376
0.777363 0.844253
0.466246 0.734079
0.682889 0.0934539
0.850937 0.977492
0.198297 0.253312
0.485346 0.68019
0.418746 0.684918];

xo = A(:,1); yo = A(:,2); nd = 20;

% Loop to scale the points
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

% Estimate the gradients with a least-squares polynomial of degree d
%   based on data xd,yd,zd
 d = 3; [derx,dery,A] = derestlsqv(d,xd,yd,zd,x,y);

% Compute the errors
 ex = abs(dx-derx); ey = abs(dy-dery);
 fprintf('errx = %5.2e, erry = %5.2e, cond = %5.2e \n',ex,ey,cond(A));

 fprintf('\n');
end
