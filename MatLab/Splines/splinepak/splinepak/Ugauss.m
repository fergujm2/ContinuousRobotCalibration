% Ugauss
% SplinePAK: Copyright Larry Schumaker 2014
% Compute a definite integral with a 6 point gauss quadrature rule
%   Integrates polynomials exactly up to degree 11

% Input degree of the polynomial and define the function to integrate
n = input('input n ');
g = @(x,n) x.^n;

% Choose interval over which to integrate
a = input('input left endpoint a = ');
b = input('input right endpoint b = ');
h = (b-a)/2;

% Set sample points and weights
r =  [0.966234757, 0.830604693, 0.619309593,... 
      0.380690407, 0.169395307, 0.033765243];
w = [0.171324492, 0.360761573,  0.467913935,...
     0.467913935, 0.360761573,  0.171324492];

% Compute the quadrature value
x = a*r + (1-r)*b;
quadval = h*sum(w.*g(x,n));

% Compare with the exact integral
true = (b^(n+1) - a^(n+1))/(n+1); err = abs(quadval-true);
fprintf('estimate = %10.8e, true = %10.8e, err = %5.2e\n',quadval,true,err);
