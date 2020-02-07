% Bposps
% SplinePAK: Copyright Larry Schumaker 2014
% Interpolate scattered data with a nonnegative C1 Powell-Sabin spline

% Read a triangulation and compute the triangulation lists
[no,xo,yo,nto,TRI] = readtri; triplot(TRI,xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
    vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRI] = trilists(xo,yo,TRI);
%figure; triplot(TRI,xo,yo); 

% Sample a test function
b = hilld(xo,yo);
z = b(:,1); 

% Estimate the gradients at the vertices
ig = input('Grad Options: zeros = 0, true = 1, estimate = 2');
if ig == 0 
  zx = zeros(no,1); zy = zeros(no,1);
elseif ig == 1
   zx = b(:,2); zy = b(:,3); % true derivatives
elseif ig == 2
   m = 20; de = 3;
  [zx,zy] =  derestlsq(xo,yo,z,adjstarto,vadjo,de,m);
end

% Adjust the gradients
ig = input('Adjust gradients?  Yes = 1, No = 0');
if ig > 0
  [zx,zy] = adjgradpspos(xo,yo,z,ie1o,ie2o,zx,zy);
end

% Compute the Powell-Sabin refinement and the B-coefficients
[x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,c] = ...
  ps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,z,zx,zy);

% Check C1 continuity
d = 2; c1ck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the spline on a grid 
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y); 
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the error 
e = errg(xg,yg,g,@hill);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check the minimum value of the spline on the rendering grid
mz = min(min(g));
fprintf('Minimum of spline: %5.2e \n', mz)

% Render the x-derivative
u = [1,0];
[xg,yg,g] = valspdergrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,g');  colormap(copper);
