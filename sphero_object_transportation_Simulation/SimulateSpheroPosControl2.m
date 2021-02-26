%function SimulateSpheroPosControl2 ()

% Simulates the multi sphero position control
% The purposes of this function are:
% 1. Simulates position control of shero
% 2. Simulates Object avoidances during position control
%

clear all; clc; close all;

% 1. Define robot's initial position and orientation:
pos_ini = [10,10];
ori_ini = [0];

% 2. Define desired goal positions:
goal_points = [20,60; 30,40; 40,10; 50,70];
size_goal = size(goal_points);
n_target = size_goal(1);

% 3. Define total path of the robot:
path_set = [pos_ini; goal_points];

figure(1)
plot(path_set(:,1), path_set(:,2), 'rx-');
hold on;
xlim([0 100])
ylim([0 100])

% 3. Get final goal position:
robotGoal = path_set(end,:);

% Robot's initial status:
robotCurrentPose = [pos_ini, ori_ini]';

n = 1;
speed = 0.5;
while n_target>0
    % Determines the direction of the robot
    path_robot = path_set(n+1,:) - path_set(n,:);
    unit_path_robot = path_robot/norm(path_robot);
    Robot_dir = unit_path_robot * speed;
    
    % Updates the robot's position
    Robot_pos_x = robotCurrentPose(1) + Robot_dir(1)/5;
    Robot_pos_y = robotCurrentPose(2) + Robot_dir(2)/5;
    robotCurrentPose = [Robot_pos_x, Robot_pos_y];
    
    figure(1)
    plot(path_set(:,1), path_set(:,2), 'rx-');
    hold on;
    xlim([0 100])
    ylim([0 100])
    plot(robotCurrentPose(1), robotCurrentPose(2), 'bo');
    hold off;
    
    % Check if it reached the goal
    if ((robotCurrentPose(1) < path_set(n+1,1)+0.05) && (robotCurrentPose(1) > path_set(n+1,1)-0.05))
        n_target = n_target - 1;
        n = n + 1;
    end
end
