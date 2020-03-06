% SPintsf  10/15/14
% SPAK: Copyright Larry Schumaker 2012
% Interpolates scattered data on the sphere with a C^0 linear spherical spline

%f = @(x,y,z) x.^2;
f = @(x,y,z)  x.^4 + 1.1*y.^4 + 1.3*z.^4;

% Read in a triangulation
[n,x,y,z,nt,TRI] = sreadtri;
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] =  slists(x,y,z,TRI);

% Read in the gauss quadrature data
[wq,rq,sq,tq] = quadset(25);

nq = 25; 
m = input('input parameter for refinement m = ');
%m = round(sqrt(10000/length(v1)));
tic
 intf = sphintf(x,y,z,v1,v2,v3,e1,e2,e3,ie1,f,m,nq);
toc

fprintf('integral of f = %10.8e \n',intf);
