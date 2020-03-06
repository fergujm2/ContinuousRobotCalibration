% SPintfT 10/11/14
% SPAK: Copyright Larry Schumaker 2012
% To test Gaussian quadrature for computing the integral 
%  of a function over a spherical triangle 

% Test function
f = @(x,y,z) x %.*y;

% Fix the vertices of the triangle
x = [0;1;0]; y = [0;0;1]; z = [1;0;0]; TRI(1,:) = [1 2 3];

%NOTE: True integral is pi/4

% Read in the Gaussian points and weights
nq = 25; %nq = 79;
[w,rq,sq,tq] = quadset(nq);

m = input('input level of refinement m = '); 
tic
int = sphintfT(m,w,rq,sq,tq,x,y,z,f);
toc
fprintf('error = %10.8e \n',pi/4-int);
