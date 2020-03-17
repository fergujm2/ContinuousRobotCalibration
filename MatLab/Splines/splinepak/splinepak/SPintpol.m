% SPintpol 10/11/14
% SPAK: Copyright Larry Schumaker 2012
% To test Gaussian quadrature for computing the integral 
%  of a function over a spherical triangle 

% Fix the vertices of the triangle
x = [0;1;0]; y = [0;0;1]; z = [1;0;0]; TRI(1,:) = [1 2 3];

d = 2; 
nd = (d+2)*(d+1)/2;

% Set random coefficients
co = randpts(nd);

% Read in the Gaussian points and weights
nq = 25; %nq = 79;
[wq,rq,sq,tq] = quadset(nq);

m = input('input level of refinement m = '); 
tic
int = sphintpol(d,m,wq,rq,sq,tq,x,y,z,co);
toc

fprintf('integral = %10.8e \n',int);
