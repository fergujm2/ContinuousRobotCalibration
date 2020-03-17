% Btype1.m
% SplinePAK: Copyright Larry Schumaker 2014
% Create a type-1 triangulation and store in a data file
% Writes triangulation info to a file "out"

% Input the number of grid lines in each variable
nx = input('input nx '); ny = input('input ny ');

% Create the corresponding type-1 triangulation
xmin = input('input xmin '); xmax = input('input xmax ');
ymin = input('input ymin '); ymax = input('input ymax ');

[vx,vy,TRI] = type1(nx,ny,xmin,xmax,ymin,ymax);

% Plot it
figure; triplot(TRI,vx,vy);

% Write the triangulation to a file "out' in current directory
n = nx*ny; nt = length(TRI);
fid = fopen('out','w');
fprintf(fid,'%g\n',n);
for i = 1:n
 fprintf(fid,'%g %g\n',vx(i),vy(i));
end
fprintf(fid,'%g\n',nt);
for i = 1:nt
 fprintf(fid,'%g %g %g\n',TRI(i,1),TRI(i,2),TRI(i,3));
end

