% Bqaussquad
% SplinePAK: Copyright Larry Schumaker 2014
% To test Gaussian quadrature for computing the integral 
%  of a function over a triangle 


% Set up the triangle vertices and area
x1 = 0; y1 = 0; x2 = 1; y2 = .5; x3 = .3; y3 = 1; 
area = (x2 - x1)*(y3 - y2) - (y2 - y1)*(x3 - x2)

% Pick the number of sample points
nq = 25;  % nq = 79;

% Read in the Gaussian points and weights
[w,rq,sq,tq] = quadset(nq);

% Compute the sample points
x = rq*x1 + sq*x2 + tq * x3;
y = rq*y1 + sq*y2 + tq * y3;

% Plot the sample points
figure; hold; plot([x1,x2,x3,x1],[y1,y2,y3,y1]);
plot(x,y,'LineStyle','none','Marker','.','MarkerSize',28);
axis off;

%%%%%% Test on Bernstein polynomials

% Choose the degree and set the coefficients of a B-polynomial 
% Pick one coefficient to set to 1 and set the rest to 0
d = input('input the degree of the polynomial d = ');
c = zeros(1,(d+2)*(d+1)/2);
k = input('input the index of the basis function to integrate =  '); c(k) = 1;

% Compute the integral
int = 0;
for i = 1:nq
  fi = decast(d,rq(i),sq(i),tq(i),c);
  int = int + w(i)*fi;
end
int = area*int;

% Compute the exact integral 
exact = area/choose(d+2,2);

fprintf('the exact integral is %g\n',exact);
fprintf('the approximate integral is %g\n',int);
fprintf('the error is %g\n',abs(exact-int));
