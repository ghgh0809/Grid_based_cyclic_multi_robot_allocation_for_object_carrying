%function SimulateSpheroPosControl ()

% Simulates the multi sphero position control
% The purposes of this function are:
% 1. Simulates position control of shero
% 2. Simulates Object avoidances during position control
%

clear all; clc; close all;
% 1. Define robot's initial position and orientation:
% R1:
pos_ini = [0.00  0.00];
ori_ini = pi/2; 
% R2:
pos_ini2 = [0.00  60.00];
ori_ini2 = pi/2; 

% 2. Define desired goal positions:
% R1:
goal_points = [0,60; 40,20; 100,20];
% R2:
goal_points2 = [0,120; 40,160; 100,160];

% 3. Define total path of the robot:
% R1:
path = [];
path = [pos_ini; goal_points];
% R2:
path2 = [];
path2 = [pos_ini2; goal_points2];

% Plot paths:
figure(1)
plot(path(:,1), path(:,2),'k-');
hold on;
plot(path2(:,1), path2(:,2),'b-');
xlim([0 200])
ylim([0 200])

% 3. Get final goal position:
robotGoal = path(end,:);
robotGoal2 = path2(end,:);

% Robot's initial status:
robotCurrentPose = [pos_ini, ori_ini]';
robotCurrentPose2 = [pos_ini2, ori_ini2]';

% Define characteristics of robots and controller:
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
robot.WheelRadius = 0.01;
robot.TrackWidth = 0.01;

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 10;
controller.MaxAngularVelocity = 50;
controller.LookaheadDistance = 1.3;

controller2 = controllerPurePursuit;
controller2.Waypoints = path2;
controller2.DesiredLinearVelocity = 10;
controller2.MaxAngularVelocity = 50;
controller2.LookaheadDistance = 1.3;

frameSize = robot.TrackWidth/0.8;

% Set the goal radius:
goalRadius = 1;

% Distance between initial position and the final goal position:
distanceToGoal = norm(pos_ini - robotGoal);
distanceToGoal2 = norm(pos_ini2 - robotGoal2);

sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
while( distanceToGoal > goalRadius )
    % Returns linear velociy and angular velocity:
    % v = liniear velocity
    % omega = angular velocity
    [v, omega] = controller(robotCurrentPose);
    [v2, omega2] = controller2(robotCurrentPose2);

    % Returns robot's velocity:
    % vel = [velocity in x, velocity in y, angular velocity]
    vel = derivative(robot, robotCurrentPose, [v omega]);
    vel2 = derivative(robot, robotCurrentPose2, [v2 omega2]);
    totalvel = vel + vel2;

    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    robotCurrentPose2 = robotCurrentPose2 + vel2*sampleTime;
    %robotCurrentPose(3) =  robotCurrentPose(3)*180/pi;

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    distanceToGoal2 = norm(robotCurrentPose2(1:2) - robotGoal2(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k-")
    hold on;
    plot(path2(:,1), path2(:,2),"b-")
    hold all
    
    hold on;
    % Plot robots:
    filledCircle(robotCurrentPose(1:2).',8,1000,'k');
    filledCircle(robotCurrentPose2(1:2).',8,1000,'b');
    
    d = 8;
    oriplot_y = robotCurrentPose(2) + d*sin(robotCurrentPose(3));
    oriplot_x = robotCurrentPose(1) + d*cos(robotCurrentPose(3));
    
    plot([robotCurrentPose(1), oriplot_x], [robotCurrentPose(2), oriplot_y], 'r-')
    
    
    xlim([0 200])
    ylim([0 200])
    
    waitfor(vizRate);
end

% end