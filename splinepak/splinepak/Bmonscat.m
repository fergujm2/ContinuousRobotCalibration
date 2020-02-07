% Bmonscat 
% SplinePAK: Copyright Larry Schumaker 2014
% Construct a monotone C1 cubic spline interpolating
%  monotone data at scattered data points, using a type-2 triangulation 
%  of an associated grid. Uses Powell-Sabin minimal energy as stage 1

% Read in the data points and sample a test function
[no,xo,yo] = readxy;
zo = sigmoid(xo,yo);

% Fit minimal energy Powell-Sabin spline to data
TRI = delaunay(xo,yo);
[nbo,neo,nto,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro,bdyo,...
   vadjo,eadjo,adjstarto,tadjo,tstarto,areao,TRIo] = trilists(xo,yo,TRI);
[xp,yp,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,A] = ...
  nmdsps(xo,yo,v1o,v2o,v3o,e1o,e2o,e3o,ie1o,ie2o,trilo,triro);
[c,M,t1,t2] = menps(v1o,v2o,v3o,xp,yp,zo, ...
   v1,v2,v3,e1,e2,e3,ie1,A);
ng = 51; d = 2;
[xg,yg,g] = valspgrid(d,xp,yp,v1,v2,v3,e1,e2,e3,ie1,c,ng,0,1,0,1);
figure; surfl(xg,yg,g');  colormap(copper);

% Create a grid through the data points
x = unique(sort(xo)); y = unique(sort(yo));
z = valspgrida(d,xp,yp,v1,v2,v3,e1,e2,e3,ie1,c,x,y); 

% Construct the type2 triangulation with grid lines given by x and y
[tx,ty,TRI] = type2nu(x,y);
figure; triplot(TRI,tx,ty);

% Evaluate the gradients on the grid
[zx,zy] = gradgrid(x,y,z);

% Adjust the grid values to be monotone consistent
z = monzadj(x,y,xo,yo,zo,z);

% Adjust the gradients
[zx,zy] = hsadj(x,y,z,zx,zy);

% Compute the B-coefficients of the C1 cubic spline interpolant
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
    vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(tx,ty,TRI);
c = cubsib(x,y,z,zx,zy,e1,e2,e3);

% Check the C1 continuity
d = 3;  c1ck(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);

% Evaluate the cubic spline on a grid
ng = 51; xmin = min(tx); xmax = max(tx); ymin = min(ty); ymax = max(ty);
[xg,yg,g] = valspgrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Evaluate the x-derivative on the grid
u = [1,0];
[xg,yg,gx] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
    xmin,xmax,ymin,ymax);

% Plot the x-derivative
figure; surfl(xg,yg,gx');  colormap(copper);

% Evaluate the y-derivative on the grid
u = [0,1];
[xg,yg,gy] = valspdergrid(d,tx,ty,v1,v2,v3,e1,e2,e3,ie1,c,ng,u,...
   xmin,xmax,ymin,ymax);

figure; surfl(xg,yg,gy');  colormap(copper);

% Calculate the max and RMS errors on the grid
fn = zeros(ng,ng); fnx = zeros(ng,ng); fny = zeros(ng,ng);
for i = 1:ng
 xgi = xg(i);
 for j = 1:ng
   b = sigmoider(xgi,yg(j));
   fn(i,j) = b(1); fnx(i,j) = b(2); fny(i,j) = b(3);
 end
end

e = reshape(fn - g,ng^2,1);  
RMS = sqrt(sum(sum(e.^2))/(ng*ng));
fprintf('Maximum error: %5.2e,   RMS = %5.2e\n',norm(e,inf),erms(e)); 

e = reshape(fnx - gx,ng^2,1);  
fprintf('x-Deriv, Maximum error: %5.2e,   RMS = %5.2e\n',norm(e,inf),erms(e)); 

e = reshape(fny - gy,ng^2,1);  
fprintf('y-Deriv, Maximum error: %5.2e,   RMS = %5.2e\n',norm(e,inf),erms(e));

% Check if derivatives are positive on the evaluation grid
mx = min(min(gx)); my = min(min(gy));
fprintf('The minimum of the x and y-derivs = %5.2e,  %5.2e\n', mx,my)
