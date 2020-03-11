clc;
close all;
clear all;
figure(1)
view(3)
axis([-1,1,-1,1,0,1.8]);
axis equal
grid on; 

load('puma_start_params.mat')
% frames = puma_dirkin([0 pi/3 pi/3 0 pi/6 pi/4]);
% frames = puma_dirkin([-pi/4 2*pi/3 pi/4 pi/4 -pi/6 -pi/4]);

draw_puma560(frames,'surface')
%draw_puma560(frames,'stick')