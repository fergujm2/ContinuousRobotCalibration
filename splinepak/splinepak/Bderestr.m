% Bderestr 
% SplinePAK: Copyright Larry Schumaker 2014
% Estimate gradients at all vertices of a given triangulation
% Compares local least-squares polynomials and the Gaussian RBF interpolant
%   on a rectangular grid with scaled Franke's function

lx = input('input x-scale factor lx = '); 
ly = input('input y-scale factor ly = ');
xmin = 0; xmax = lx; ymin = 0; ymax = ly;

% Input number of grid lines
nx = input('input number of x-grid lines nx = '); 
ny = input('input number of y-grid lines ny = ');

% Create a type-1 triangulation
[x,y,TRI] = type1(nx,ny,xmin,xmax,ymin,ymax);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
   vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Sample Franke's function
der = franke2d(x./lx,y./ly); 
z = der(:,1); zxl = der(:,2)./lx; zyl = der(:,3)./ly;

m = 20; d = 3; eps = 3; 

ic = input('input 0 = lsq, 1 = lsqk, 2 = rbf, 3 = rbfk ');
if ic == 0
tic
[zx,zy] = derestlsq(x,y,z,adjstart,vadj,d,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));

elseif ic == 1
tic
[zx,zy] = derestlsqk(x,y,z,d,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));

elseif ic == 2
tic
[zx,zy] = derestrbf(x,y,z,adjstart,vadj,eps,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));

else
tic
[zx,zy] = derestrbfk(x,y,z,eps,m);
toc
ex = zx-zxl; ey  = zy - zyl;
fprintf('Max and RMS errors fx:  %5.2e %5.2e \n', norm(ex,inf),erms(ex));
fprintf('Max and RMS errors fy:  %5.2e %5.2e \n\n', norm(ey,inf),erms(ey));
end

