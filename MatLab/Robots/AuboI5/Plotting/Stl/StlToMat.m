h = figure(1);
clf;
h.Color = [1,1,1];
hold on;
daspect([1,1,1]);
view([120,30]);
xlabel('X');
ylabel('Y');
zlabel('Z');
lighting flat
light('Position', [100,100,100]);
material('shiny');      

draw_coordinate_system(0.125, eye(3), zeros(3,1), ['r' 'g' 'b'], 'W');

%% Base

[V, F] = stlReadBinary('base.STL');
n = size(V, 1);
C = ones(1, n)*0.5;
trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/base.mat', 'V', 'F');

%% Shoulder

[V, F] = stlReadBinary('s_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

T = axang2tform([0, 0, 1, pi])*axang2tform([1, 0, 0, pi/2]);
V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/shoulder_parts.mat', 'V', 'F');

[V, F] = stlReadBinary('shoulder_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/shoulder.mat', 'V', 'F');

%% Upper Arm

[V, F] = stlReadBinary('f1_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

T = trvec2tform([-0.408, 0, 0])*axang2tform([1, 0, 0, pi]);
V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/upper_parts.mat', 'V', 'F');

[V, F] = stlReadBinary('upperArm_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/upper.mat', 'V', 'F');

%% Forearm

[V, F] = stlReadBinary('foreArm_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

T = axang2tform([1, 0, 0, pi])*trvec2tform([-0.376, 0, -0.1215]);
V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/forearm.mat', 'V', 'F');

[V, F] = stlReadBinary('u_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/forearm_parts.mat', 'V', 'F');

%% Wrist 1

[V, F] = stlReadBinary('w1_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

T = axang2tform([0, 0, 1, pi])*axang2tform([1, 0, 0, pi/2]);
V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/wrist1_parts.mat', 'V', 'F');

[V, F] = stlReadBinary('wrist1_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/wrist1.mat', 'V', 'F');

%% Wrist 2

[V, F] = stlReadBinary('w2_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

T = axang2tform([0, 0, 1, pi])*axang2tform([1, 0, 0, -pi/2]);
V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/wrist2_parts.mat', 'V', 'F');

[V, F] = stlReadBinary('wrist2_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;

V = hom2cart((T*(cart2hom(V)'))');

trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/wrist2.mat', 'V', 'F');

%% Wrist 3

[V, F] = stlReadBinary('wrist3_Link.STL');
n = size(V, 1);
C = ones(1, n)*0.5;
trisurf(F, V(:,1), V(:,2), V(:,3), C, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'AmbientStrength', 0.15);

save('../Mat/wrist3.mat', 'V', 'F');