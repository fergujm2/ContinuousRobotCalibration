% Btype2
% SplinePAK: Copyright Larry Schumaker 2014
% Create a type-1 triangulation and store in a data file
% Also write triangulation info to a file "out"

% Input the number of grid lines in each variable
nx = input('input nx '); ny = input('input ny ');

% Create the corresponding type-2 triangulation
xmin = 0; xmax = 1; ymin = 0; ymax = 1;
[vx,vy,TRI] = type2(nx,ny,xmin,xmax,ymin,ymax);

% Plot it
triplot(TRI,vx,vy);

% Determine the number of vertices and triangles
nt = length(TRI); n = nx*ny + (nx-1)*(ny-1);

% Write the triangulation to a file "out' in current directory
fid = fopen('out','w');
fprintf(fid,'%g\n',n);
for i = 1:n
 fprintf(fid,'%g %g\n',vx(i),vy(i));
end
fprintf(fid,'%g\n',nt);
for i = 1:nt
 fprintf(fid,'%g %g %g\n',TRI(i,1),TRI(i,2),TRI(i,3));
end

