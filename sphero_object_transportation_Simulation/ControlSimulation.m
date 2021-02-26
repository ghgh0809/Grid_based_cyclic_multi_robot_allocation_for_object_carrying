function Simulation (obj_boundary, centroid, gap, path_choice)
% Simulates the transportation of the object
% 

%% Initialize the position of the object:
obj_boundary_Lpos = obj_boundary;
centroid_path = centroid;

% Set path and speed of the transportation:
% Assumption: 1. speed is uniform
%             2. ignore friction
%
speed  = 1;

%% Path1: y = 2x
if (path_choice(1) == 1)
    path_x = 0:speed:200;
    path_y = path_x * 2;
    path = [path_x.', path_y.'];
    [obj_boundary_Lpos centroid_path_addon] = SimulateSpheroPosControl4(obj_boundary_Lpos, centroid, centroid_path, gap, path);
    centroid_path = [centroid_path; centroid_path_addon];
end

%% Path2: y = -x
if (path_choice(2) == 1)
    path_x = 0:speed:100;
    path_y = path_x * -1;
    path = [path_x.', path_y.'];
    [obj_boundary_Lpos centroid_path_addon] = SimulateSpheroPosControl4(obj_boundary_Lpos, centroid, centroid_path, gap, path);
    centroid_path = [centroid_path; centroid_path_addon];
end

%% Path3: y = 1
if (path_choice(3) == 1)
    path_y = 0:speed*1:100;
    path_x = path_y * 0;
    path = [path_x.', path_y.'];
    [obj_boundary_Lpos centroid_path_addon] = SimulateSpheroPosControl4(obj_boundary_Lpos, centroid, centroid_path, gap, path);
    centroid_path = [centroid_path; centroid_path_addon];
end

%% Path4: x = 1
if (path_choice(4) == 1)
    path_x = 0:speed:200;
    path_y = path_x * 0;
    path = [path_x.', path_y.'];
    [obj_boundary_Lpos centroid_path_addon] = SimulateSpheroPosControl4(obj_boundary_Lpos, centroid, centroid_path, gap, path);
    centroid_path = [centroid_path; centroid_path_addon];
end

%% Path5: y = sin(x)
if (path_choice(5) == 1)
    path_x = 0:1:100*pi;
    path_y = 100*sin(path_x/100);
    path = [path_x.', path_y.'];
    [obj_boundary_Lpos centroid_path_addon] = SimulateSpheroPosControl4(obj_boundary_Lpos, centroid, centroid_path, gap, path);
    centroid_path = [centroid_path; centroid_path_addon];
end