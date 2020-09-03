function DrawRobot(frames)
% Set material options
lighting flat
light('Position', [100,100,100]);
material('shiny');

auboBlack = [0.25098 0.25098 0.25098];
auboOrange = [1 0.65098 0];


persistent meshes

if isempty(meshes)
    meshes.base = load('base');
    meshes.shoulder = load('shoulder');
    meshes.shoulderParts = load('shoulder_parts');
    meshes.upper = load('upper');
    meshes.upperParts = load('upper_parts');
    meshes.forearm = load('forearm');
    meshes.forearmParts = load('forearm_parts');
    meshes.wrist1 = load('wrist1');
    meshes.wrist1Parts = load('wrist1_parts');
    meshes.wrist2 = load('wrist2');
    meshes.wrist2Parts = load('wrist2_parts');
    meshes.wrist3 = load('wrist3');
end

% Draw Base
V = frames(:,:,1)*(cart2hom(meshes.base.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.base.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboBlack, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% draw_coordinate_system(0.2, frames(1:3,1:3,1), frames(1:3,4,1), ['r' 'g' 'b'], 'B');

% Draw Shoulder
V = frames(:,:,2)*(cart2hom(meshes.shoulder.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.shoulder.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboOrange, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

V = frames(:,:,2)*(cart2hom(meshes.shoulderParts.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.shoulderParts.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboBlack, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% draw_coordinate_system(0.2, frames(1:3,1:3,2), frames(1:3,4,2), ['r' 'g' 'b'], '1');

% Draw Upper Arm
V = frames(:,:,3)*(cart2hom(meshes.upper.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.upper.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboOrange, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

V = frames(:,:,3)*(cart2hom(meshes.upperParts.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.upperParts.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboBlack, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% draw_coordinate_system(0.2, frames(1:3,1:3,3), frames(1:3,4,3), ['r' 'g' 'b'], '2');

% Draw Forearm
V = frames(:,:,4)*(cart2hom(meshes.forearm.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.forearm.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboOrange, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

V = frames(:,:,4)*(cart2hom(meshes.forearmParts.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.forearmParts.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboBlack, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% draw_coordinate_system(0.075, frames(1:3,1:3,4), frames(1:3,4,4), ['r' 'g' 'b'], '3');

% Draw Wrist 1
V = frames(:,:,5)*(cart2hom(meshes.wrist1.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.wrist1.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboOrange, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

V = frames(:,:,5)*(cart2hom(meshes.wrist1Parts.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.wrist1Parts.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboBlack, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% draw_coordinate_system(0.075, frames(1:3,1:3,5), frames(1:3,4,5), ['r' 'g' 'b'], '4');

% Draw Wrist 2
V = frames(:,:,6)*(cart2hom(meshes.wrist2.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.wrist2.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboOrange, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

V = frames(:,:,6)*(cart2hom(meshes.wrist2Parts.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.wrist2Parts.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboBlack, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

% draw_coordinate_system(0.075, frames(1:3,1:3,6), frames(1:3,4,6), ['r' 'g' 'b'], '5');

% Draw End Effector
V = frames(:,:,7)*(cart2hom(meshes.wrist3.V)');
n = length(V(3,:));
C = ones(1, n)*0.5;
trisurf(meshes.wrist3.F, V(1,:), V(2,:), V(3,:), C, 'FaceColor', auboBlack, 'EdgeColor', 'none', 'AmbientStrength', 0.5);

draw_coordinate_system(0.085, frames(1:3,1:3,7), frames(1:3,4,7), ['r' 'g' 'b']);

end
