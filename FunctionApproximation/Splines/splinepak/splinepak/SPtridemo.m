% SPtridemo
% SplinePAK: Copyright Larry Schumaker 2014
% Demonstrate how to handle spherical triangulations

% Read in a spherical triangulation
[n,x,y,z,nt,TRI] = sreadtri;

% Compute the triangulation lists
[v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir] = slists(x,y,z,TRI);

% Render the triangulation
tic
srendtri(x,y,z,ie1,ie2); 
toc

return

% Compute the area of each triangle
area = zeros(nt,1);
for i = 1:nt
  v1i = v1(i); v2i = v2(i); v3i = v3(i);
  area(i) = sarea(x(v1i),y(v1i),z(v1i),...
       x(v2i),y(v2i),z(v2i),x(v3i),y(v3i),z(v3i));
end

% Check the sum of the areas -- should be 4 pi
sumarea = sum(area)

%%%%

% Test integration
nq = 25; [wq,rq,sq,tq] =  quadset(nq);
nf = input('input nf ');
val = 0; sumarea = 0;
for i  = 1:nt
  v1i = v1(i);   v2i = v2(i);   v3i = v3(i);
  x1 = x(v1i);  y1 = y(v1i);  z1 = z(v1i);
  x2 = x(v2i);  y2 = y(v2i);  z2 = z(v2i);
  x3 = x(v3i);  y3 = y(v3i);  z3 = z(v3i);
  ai = sarea(x1,y1,z1,x2,y2,z2,x3,y3,z3);
  sumarea = sumarea + ai;
   xk = rq*x1 + sq*x2 + tq*x3;
   yk = rq*y1 + sq*y2 + tq*y3;
   zk = rq*z1 + sq*z2 + tq*z3;
   [xk,yk,zk] = unit(xk,yk,zk);
   fk = sfun(nf,xk,yk,zk);
   val = val + ai*sum(wq.*fk);
end
fprintf('integral over the sphere = %g \n ',val)

% Note: should get 4*pi if use nf = 2 (the constant function)
