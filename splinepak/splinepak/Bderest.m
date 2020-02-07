% Bderest 
% SplinePAK: Copyright Larry Schumaker 2014
% Estimate gradients at all vertices of a given triangulation
% Compares derestlsq, derestlsqk, derestrbf, and derestrbfk

% Read in a triangulation
[n,x,y,nt,TRI] = readtri;   %figure; triplot(TRI,x,y);

% Compute the triangulation lists
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Sample a test function
der = franke2d(x,y);  z = der(:,1); zxl = der(:,2); zyl = der(:,3);

% Test with a linear function
%z = x+y; zxl = ones(n,1); zyl = ones(n,1);


% Set the degree of the LSQ polynomial and number of points to use
%d = 2; m = 12;
d = 3; m = 20; eps = 4;

tic
[zx,zy] = derestlsq(x,y,z,adjstart,vadj,d,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('derestlsq \n');
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));

tic
[zx,zy] = derestlsqk(x,y,z,d,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('derestlsqk \n');
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));

tic
[zx,zy] = derestrbf(x,y,z,adjstart,vadj,eps,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('derestrbf \n');
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));

tic
[zx,zy] = derestrbfk(x,y,z,eps,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('derestrbf \n');
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));

