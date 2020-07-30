function DrawRobot(frames)

% Draw Base
obj = load('puma560_base_meters');
base = frames(:,:,1)*obj.base;
tri = obj.t;

n = length(base(3,:));
C = ones(1, n)*0.5;

trisurf(tri, base(1,:), base(2,:), base(3,:), C, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% Draw Link 1
obj = load('puma560_link1_meters');
link1 = frames(:,:,2)*obj.link1;
tri = obj.t;

n = length(link1(3,:));
C = ones(1, n)*0.5;

trisurf(tri, link1(1,:), link1(2,:), link1(3,:), C, 'FaceColor', [1 0 0], 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% Draw Link 2
obj = load('puma560_link2_meters');
link2 = frames(:,:,3)*obj.link2;
tri = obj.t;

n = length(link2(3,:));
C = ones(1, n)*0.5;

trisurf(tri, link2(1,:), link2(2,:), link2(3,:), C, 'FaceColor', [1 0.5 0], 'EdgeColor', 'none');

% Draw Link 3
obj = load('puma560_link3_meters');
link3 = frames(:,:,4)*obj.link3;
tri = obj.t;

n = length(link3(3,:));
C = ones(1, n)*0.5;

trisurf(tri, link3(1,:), link3(2,:), link3(3,:), C, 'FaceColor', [1, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

% Draw Link 4
obj = load('puma560_link4_meters');
link4 = frames(:,:,5)*obj.link4;
tri = obj.t;

n = length(link4(4,:));
C = ones(1, n)*0.5;

trisurf(tri, link4(1,:), link4(2,:), link4(3,:), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

% Draw Link 5
obj = load('puma560_link5_meters');
link5 = frames(:,:,6)*obj.link5;
tri = obj.t;

n = length(link5(4,:));
C = ones(1, n)*0.5;

trisurf(tri, link5(1,:), link5(2,:), link5(3,:), C, 'FaceColor', [0, 0, 1], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

% Draw Link 6
obj = load('puma560_link6_meters');
link6 = frames(:,:,7)*obj.link6;
tri = obj.t;

n = length(link6(4,:));
C = ones(1, n)*0.5;

trisurf(tri, link6(1,:), link6(2,:), link6(3,:), C, 'FaceColor', [0 0 1], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

% Draw pen
obj = load('markerpen_meters');
pen = frames(:,:,7)*obj.pen;
tri = obj.t;

n = length(pen(4,:));
C = ones(1, n)*0.5;

trisurf(tri, pen(1,:), pen(2,:), pen(3,:), C, 'FaceColor', [226/255, 26/255, 91/255], 'EdgeColor', 'None', 'AmbientStrength', 0.15);

% Set material options
lighting flat
light('Position', [100,100,100]);
material('shiny');
end
