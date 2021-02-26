%function SimulateSpheroPosControl2 ()

% Simulates the multi sphero position control
% The purposes of this function are:
% 1. Simulates position control of shero
% 2. Simulates Object avoidances during position control
%

clear all; clc; close all;

% 1. Define robot's initial position and orientation:
pos_ini1 = [10,10];
pos_ini2 = [10,20];
pos_ini3 = [10,30];
pos_ini4 = [10,40];

ori_ini = [0];

% 2. Define desired goal positions:
goal_points1 = [20,10; 30,10; 40,10; 50,10];
goal_points2 = [20,20; 30,20; 40,20; 50,20];
goal_points3 = [20,30; 30,30; 40,30; 50,30];
goal_points4 = [10,50; 20,50; 20,40; 30,40; 30,50; 40,50; 40,40];

n_target1 = size(goal_points1, 1);
n_target2 = size(goal_points2, 1);
n_target3 = size(goal_points3, 1);
n_target4 = size(goal_points4, 1);

% 3. Define total path of the robot:
path_set1 = [pos_ini1; goal_points1];
path_set2 = [pos_ini2; goal_points2];
path_set3 = [pos_ini3; goal_points3];
path_set4 = [pos_ini4; goal_points4];

% Robot's initial status:
robotCurrentPose1 = [pos_ini1, ori_ini]';
robotCurrentPose2 = [pos_ini2, ori_ini]';
robotCurrentPose3 = [pos_ini3, ori_ini]';
robotCurrentPose4 = [pos_ini4, ori_ini]';

n1 = 1; n2 = 1; n3 = 1; n4 = 1;
speed1 = 0.2;
speed2 = 0.4;
speed3 = 0.2;
speed4 = 0.4;
while ((n_target1 > 0)||(n_target2 > 0)||(n_target3 > 0)||(n_target4 > 0))
    if (n_target1>0)
        % Determines the direction of the robot1
        path_robot1 = path_set1(n1+1,:) - path_set1(n1,:);
        unit_path_robot1 = path_robot1/norm(path_robot1);
        Robot_dir1 = unit_path_robot1 * speed1;
        % Updates the robot's position
        Robot_pos_x1 = robotCurrentPose1(1) + Robot_dir1(1);
        Robot_pos_y1 = robotCurrentPose1(2) + Robot_dir1(2);
        robotCurrentPose1 = [Robot_pos_x1, Robot_pos_y1];
        
        if ((robotCurrentPose1(1) < path_set1(n1+1,1)+0.1) && (robotCurrentPose1(1) > path_set1(n1+1,1)-0.1) && (robotCurrentPose1(2) < path_set1(n1+1,2)+0.1) && (robotCurrentPose1(2) > path_set1(n1+1,2)-0.1))
            if (n_target1>0)
                n_target1 = n_target1 - 1;
                n1 = n1 + 1;
            end
        end
    end
    
    if (n_target2>0)
        % Determines the direction of the robot2
        path_robot2 = path_set2(n2+1,:) - path_set2(n2,:);
        unit_path_robot2 = path_robot2/norm(path_robot2);
        Robot_dir2 = unit_path_robot2 * speed2;
        % Updates the robot's position
        Robot_pos_x2 = robotCurrentPose2(1) + Robot_dir2(1);
        Robot_pos_y2 = robotCurrentPose2(2) + Robot_dir2(2);
        robotCurrentPose2 = [Robot_pos_x2, Robot_pos_y2];
        if ((robotCurrentPose2(1) < path_set2(n2+1,1)+0.1) && (robotCurrentPose2(1) > path_set2(n2+1,1)-0.1) && (robotCurrentPose2(2) < path_set2(n2+1,2)+0.1) && (robotCurrentPose2(2) > path_set2(n2+1,2)-0.1))
            if (n_target2>0)
                n_target2 = n_target2 - 1;
                n2 = n2 + 1;
            end
        end
    end
    
    if (n_target3>0)
        % Determines the direction of the robot3
        path_robot3 = path_set3(n3+1,:) - path_set3(n3,:);
        unit_path_robot3 = path_robot3/norm(path_robot3);
        Robot_dir3 = unit_path_robot3 * speed3;
        % Updates the robot's position
        Robot_pos_x3 = robotCurrentPose3(1) + Robot_dir3(1);
        Robot_pos_y3 = robotCurrentPose3(2) + Robot_dir3(2);
        robotCurrentPose3 = [Robot_pos_x3, Robot_pos_y3];
        if ((robotCurrentPose3(1) < path_set3(n3+1,1)+0.1) && (robotCurrentPose3(1) > path_set3(n3+1,1)-0.1) && (robotCurrentPose3(2) < path_set3(n3+1,2)+0.1) && (robotCurrentPose3(2) > path_set3(n3+1,2)-0.1))
            if (n_target3>0)
                n_target3 = n_target3 - 1;
                n3 = n3 + 1;
            end
        end
    end
    
    if (n_target4>0)
        % Determines the direction of the robot4
        path_robot4 = path_set4(n4+1,:) - path_set4(n4,:);
        unit_path_robot4 = path_robot4/norm(path_robot4);
        Robot_dir4 = unit_path_robot4 * speed4;
        % Updates the robot's position
        Robot_pos_x4 = robotCurrentPose4(1) + Robot_dir4(1);
        Robot_pos_y4 = robotCurrentPose4(2) + Robot_dir4(2);
        robotCurrentPose4 = [Robot_pos_x4, Robot_pos_y4];
        if ((robotCurrentPose4(1) < path_set4(n4+1,1)+0.1) && (robotCurrentPose4(1) > path_set4(n4+1,1)-0.1) && (robotCurrentPose4(2) < path_set4(n4+1,2)+0.1) && (robotCurrentPose4(2) > path_set4(n4+1,2)-0.1))
            if (n_target4>0)
                n_target4 = n_target4 - 1;
                n4 = n4 + 1;
            end
        end
    end
    
    figure(1)
    plot(path_set1(:,1), path_set1(:,2), 'rx-');
    hold on;
    plot(path_set2(:,1), path_set2(:,2), 'rx-');
    plot(path_set3(:,1), path_set3(:,2), 'rx-');
    plot(path_set4(:,1), path_set4(:,2), 'rx-');
    xlim([0 100])
    ylim([0 100])
    plot(robotCurrentPose1(1), robotCurrentPose1(2), 'bo');
    plot(robotCurrentPose2(1), robotCurrentPose2(2), 'bo');
    plot(robotCurrentPose3(1), robotCurrentPose3(2), 'bo');
    plot(robotCurrentPose4(1), robotCurrentPose4(2), 'bo');
    hold off;
    
end
