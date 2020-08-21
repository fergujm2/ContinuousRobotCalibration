% Brendpol.m
% SplinePAK: Copyright Larry Schumaker 2014
% Renders a polynomial patch on a triangle

% Set the vertices of the triangle
x1 = 0; y1 = 0; x2 = 1; y2 = 0; x3 = 0; y3 = 1;

% Choose the degree
d = 3; 

% Compute the number of coefs
nd = (d+2)*(d+1)/2;

% Set one coef to one, the rest to zero
c = zeros(nd); c(3) = 1;

% Input m to determine how many subtriangles to render

m = input('input m ');
[gx,gy,gz,gTRI] = rendpol(d,m,x1,y1,x2,y2,x3,y3,c);

% Plot the surface
figure; h = trisurf(gTRI,gx,gy,gz);
set(h,'FaceColor',[0 1 1])


