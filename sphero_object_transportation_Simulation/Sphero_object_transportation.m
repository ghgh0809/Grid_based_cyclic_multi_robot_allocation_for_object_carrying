% This is the main Matlab file to simulate the object transportation
% using sphero. The spheros will carry the object and move to desired 
% goal point. 
% The object needs to be defined prior of running the file.
% 
% Title : Grid-based Cyclic Robot Allocation for Object Carrying
% Writer: Jee Hwan Park

clear; clc; close all;
addpath('geom2d')
addpath('object')
warning('off','all')
warning

%% Define object properties:
I = imread('object5_circle.png'); % Define object
[obj_boundary, centroid] = ObjectDefine(I);

%% Compute the gap distance:
gap = GapDefine(obj_boundary, centroid);

% For exceptional solution:
% g_x = gap(1);
% g_y = gap(2);
% if (g_x < g_y)
%     gap(2) = g_x;
% else
%     gap(1) = g_y;
% end

%% Draw grid:
Drawgrid(obj_boundary, centroid, gap);

%% Simulating Transportion of the object:
% Followings are the path options. Please enable desired path (ordered)
% Path1: y = 2x
% Path2: y = -x
% Path3: y = 1
% Path4: x = 1
% Path5: y = sin(x)
path_choice = [0 0 0 0 1];

Simulation(obj_boundary, centroid, gap, path_choice);
