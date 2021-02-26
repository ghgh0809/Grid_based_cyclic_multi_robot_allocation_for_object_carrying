function [obj_boundary_Lpos, centroid_path] = SimulateTransport (obj_boundary, centroid, centroid_path_in, gap, path)
% Simulates the transportation of the object
% The function has no returns
% The simulation includes followings:
% 1. Object movement (o)
% 2. Centroid path (o)
% 3. Robot positions (o)
% 4. boundary of under robots (o)
% 5. Stability status
% 6. Robot paths
% 7. Final number of robots used (hmm?)
% 

%% Initialization of some variables:
% For object draw:
obj_boundary_Lpos = obj_boundary;
polyin = polyshape(obj_boundary(:,1), obj_boundary(:,2));

% For path draw:
centroid_path_x = centroid_path_in(end,1);
centroid_path_y = centroid_path_in(end,2);
path_x_size = size(path(:,1));
path_y_size = size(path(:,2));
path_x = path(:,1).';
path_y = path(:,2).';
limits = [2000 2000];

% For gap draw:
temp_div = centroid(1)/gap(1);
start_x = centroid(1) - 2*(floor(temp_div)*gap(1));
temp_div = centroid(2)/gap(2);
start_y = centroid(2) - 2*(floor(temp_div)*gap(2));
grid_xticks = round(start_x,2):round(gap(1),2):limits(1);
grid_yticks = round(start_y,2):round(gap(2),2):limits(2);

% For robot position:
all_cross_pos = [];
size_grid_xticks = size(grid_xticks);
size_grid_yticks = size(grid_yticks);
for i=1:1:size_grid_xticks(2)
    for j=1:1:size_grid_yticks(2)
        all_cross_pos = [all_cross_pos; grid_xticks(i), grid_yticks(j)];
    end
end

%% Simulate the movement of the object with centroid:
% Define the iteration number:
if (path_x_size(1)>path_y_size(1)) no_i = path_x_size(1);
else no_i = path_y_size(1); end
max_no_robot = 0;
robot_xy_total=[];
for i_x = 1:1:no_i
    % Iteration settings:
    i_y = i_x;
    if (i_x >= path_x_size(1)) i_x = path_x_size(1); end
    if (i_y >= path_y_size(1)) i_y = path_y_size(1); end
    
    % Define new polygon for each iteration:
    polyin2 = polyshape(obj_boundary(:,1)+path_x(i_x), obj_boundary(:,2)+path_y(i_y));
    
    % Define new centroid for each iteration:
    [ geom, iner, cpmo ] = polygeom(obj_boundary(:,1)+path_x(i_x), obj_boundary(:,2)+path_y(i_y));
    centroid_x_new = geom(2);   centroid_y_new = geom(3);
    
    % Store the last/recent position of the object:
    obj_boundary_Lpos = [obj_boundary(:,1)+path_x(i_x), obj_boundary(:,2)+path_y(i_y)];
    
    % Define the path of the centroid of the object:
    centroid_path_x = [centroid_path_x, centroid_x_new];
    centroid_path_y = [centroid_path_y, centroid_y_new];
    % centroid_path = [centroid_path_x; centroid_path_y].';
    centroid_path_temp = [centroid_path_x; centroid_path_y].';
    centroid_path = [centroid_path_in; centroid_path_temp];
    
    % Compute position and boundary of robots (Under):
    in = inpolygon(all_cross_pos(:,1), all_cross_pos(:,2), obj_boundary_Lpos(:,1), obj_boundary_Lpos(:,2));
    no_in = sum(in(:) == 1);
    all_cross_pos_x = all_cross_pos(:,1).';
    all_cross_pos_y = all_cross_pos(:,2).';
    robot_u_pos_x = all_cross_pos_x(in);
    robot_u_pos_y = all_cross_pos_y(in);
    robot_u_boundary = boundary(robot_u_pos_x.', robot_u_pos_y.', 0.1);
    robot_u_polygon = polyshape(robot_u_pos_x(robot_u_boundary), robot_u_pos_y(robot_u_boundary));
    robot_u_xy = [all_cross_pos_x(in).', all_cross_pos_y(in).'];
    
    % Compute position and boundary of robots (Surround):
    robot_s_xy = [];
    add_value = round([gap(1),0; gap(1)*-1,0; 0,gap(2); 0,gap(2)*-1; gap(1),gap(2); gap(1)*-1,gap(2); gap(1),gap(2)*-1; gap(1)*-1,gap(2)*-1], 2);
    for (i=1:size(robot_u_xy,1))
        x = robot_u_xy(i,1);
        y = robot_u_xy(i,2);
        for (j=1:size(add_value,1))
            new_x = x+add_value(j,1);
            new_y = y+add_value(j,2);
            result = redundant_check(robot_u_xy, robot_s_xy, new_x, new_y);
            if result == 1
                robot_s_xy = [robot_s_xy; new_x, new_y];
            end
        end
    end
    
    % Compute starting position of robots:
    if (i_x == 1)
        robot_u_xy_start = robot_u_xy;
        robot_s_xy_start = robot_s_xy;
    end
    
    % Compute all positions covered during the transportation:
    robot_xy_total = [robot_xy_total; robot_s_xy; robot_u_xy];
    robot_xy_total_table = table(robot_xy_total);
    robot_xy_total_table = unique(robot_xy_total_table);
    robot_xy_total = table2array(robot_xy_total_table);
    
    sum_of_no_robot = size(robot_s_xy,1) + size(robot_u_xy,1);
    if sum_of_no_robot > max_no_robot
        max_no_robot = sum_of_no_robot;
    end
end
% max_no_robot
% robot_xy_total

figure(2)
drawplots(polyin, polyin2, centroid, centroid_x_new, centroid_y_new, centroid_path, limits, grid_xticks, grid_yticks, all_cross_pos, in, robot_u_polygon, robot_xy_total, robot_u_xy_start, robot_s_xy_start);

%% Now control the position of each robot:
% Now we are using limited number of robots to transport the object
% if (path_x_size(1)>path_y_size(1)) no_i = path_x_size(1);
% else no_i = path_y_size(1); end
% robot_t_xy_start = [robot_u_xy_start; robot_s_xy_start];
% robot_old_u = robot_u_xy_start;
% for i_x = 1:1:no_i
%     % Iteration settings:
%     i_y = i_x;
%     if (i_x >= path_x_size(1)) i_x = path_x_size(1); end
%     if (i_y >= path_y_size(1)) i_y = path_y_size(1); end
%     
%     % New positon 
%     robot_new_xy = [robot_t_xy_start(:,1)+path_x(i_x), robot_t_xy_start(:,2)+path_y(i_y)];
%     robot_new_xy_x = robot_new_xy(:,1).';
%     robot_new_xy_y = robot_new_xy(:,2).';
%     
%     % Compute the velocity of the robot (under):
%     % robot_displace_u = robot_new_u - robot_old_u
%     
%     % Define new polygon for each iteration:
%     polyin2 = polyshape(obj_boundary(:,1)+path_x(i_x)+path_x(i_x), obj_boundary(:,2)+path_y(i_y)+path_y(i_y));
%     
%     % Define new centroid for each iteration:
%     [ geom, iner, cpmo ] = polygeom(obj_boundary(:,1)+path_x(i_x)+path_x(i_x), obj_boundary(:,2)+path_y(i_y)+path_y(i_y));
%     centroid_x_new = geom(2);   centroid_y_new = geom(3);
%     
%     % Store the last/recent position of the object:
%     obj_boundary_Lpos = [obj_boundary(:,1)+path_x(i_x)+path_x(i_x), obj_boundary(:,2)+path_y(i_y)+path_y(i_y)];
%     
%     % Compute pos of robot (under)
%     in = inpolygon(robot_new_xy(:,1), robot_new_xy(:,2), obj_boundary_Lpos(:,1), obj_boundary_Lpos(:,2));
%     robot_new_u_x = robot_new_xy_x(in);
%     robot_new_u_y = robot_new_xy_y(in);
%     robot_new_u = [robot_new_u_x,robot_new_u_y];
%     robot_u_boundary = boundary(robot_new_u_x.', robot_new_u_y.', 0.1);
%     robot_u_polygon = polyshape(robot_new_u_x(robot_u_boundary), robot_new_u_y(robot_u_boundary))
%     
%     % Check if the COM is within the polygon:
%     in_check = inpolygon(centroid_x_new, centroid_y_new, robot_new_u_x(robot_u_boundary), robot_new_u_y(robot_u_boundary));
%     if (in_check)
%         status = 1;
%     else
%         status = 0;
%     end
%     
%     figure(3)
%     drawplots2(limits, grid_xticks, grid_yticks, polyin, centroid, robot_new_xy, in, polyin2, centroid_x_new, centroid_y_new, robot_u_polygon, status);
%     
%     robot_old_u = robot_new_u;
% end

end

%% 'drawplots' function to plot object, centroid, path of centroid:
function drawplots(polyin, polyin2, centroid, centroid_x_new, centroid_y_new, centroid_path, limits, grid_xticks, grid_yticks, all_cross_pos, in, robot_u_polygon, robot_xy_total, robot_u_xy_start, robot_s_xy_start)
    all_cross_pos_x = all_cross_pos(:,1).';
    all_cross_pos_y = all_cross_pos(:,2).';
    
    % Plot object:
    plot(polyin);
    hold on;
    % plot(polyin2);
    
    % Draw grid:
    drawgrid(grid_xticks, grid_yticks);
    
    % Plot centroid:
    plot(centroid(1), centroid(2),'rx','Linewidth',2, 'markersize',13); % Starting position
    % plot(centroid_x_new, centroid_y_new,'rx','Linewidth',2, 'markersize',13); % New position
    
    % Plot path of centroid:
    % line(centroid_path(:,1), centroid_path(:,2),'Color','red');
    
    % Plot all possible pos of robots:
    % plot(all_cross_pos_x(in), all_cross_pos_y(in),'r+'); % points inside
    % plot(all_cross_pos_x(~in), all_cross_pos_y(~in),'bo'); % points outside
    
    % Plot surrounding robots:
    % plot(robot_s_pos(:,1), robot_s_pos(:,2),'bo');
    
    % Plot boundary of robots(under):
    % plot(robot_u_polygon,'FaceColor','red','FaceAlpha',0.1);
    
    % Plot all possible positon of robots:
    plot(robot_xy_total(:,1), robot_xy_total(:,2),'bo');
    
    % Plot starting position of robots:
    plot(robot_u_xy_start(:,1), robot_u_xy_start(:,2),'ko');
    plot(robot_s_xy_start(:,1), robot_s_xy_start(:,2),'ro');
    
    % Plot new position of robots (with only starting robots):
    plot(robot_new_xy_start(:,1), robot_new_xy_start(:,2),'ko');
    
    xlim([-100 limits(1)]); ylim([-100 limits(2)]);
    drawnow;
    hold off;
end

%% 'drawplots2' function to plot object, centroid, path of centroid:
function drawplots2(limits, grid_xticks, grid_yticks, polyin, centroid, robot_new_xy, in, polyin2, centroid_x_new, centroid_y_new, robot_u_polygon, status)
    % Plot object:
    % plot(polyin);
    plot(polyin2);
    hold on;
    
    % Draw grid:
    % drawgrid(grid_xticks, grid_yticks);
    
    % Plot robot under the object
    robot_new_xy_x = robot_new_xy(:,1);
    robot_new_xy_y = robot_new_xy(:,2);
    plot(robot_new_xy_x(in), robot_new_xy_y(in),'r+'); % points inside
    plot(robot_new_xy_x(~in), robot_new_xy_y(~in),'bo'); % points outside
    
    % Plot centroid:
    % plot(centroid(1), centroid(2),'rx','Linewidth',2, 'markersize',13); % Starting position
    plot(centroid_x_new, centroid_y_new,'rx','Linewidth',2, 'markersize',13); % New position
    
    % Plot new position of robots (with only starting robots):
    plot(robot_new_xy(:,1), robot_new_xy(:,2),'ko');
    
    % Plot polygon formed by robots:
    plot(robot_u_polygon);
    
    xlim([-100 limits(1)]); ylim([-100 limits(2)]);
    drawnow;
    hold off;
end

%% 'drawgrid' functon to draw grid:
function drawgrid(grid_xticks, grid_yticks)
    grid on;
    
    % Grid properties:
    ax = gca;
    ax.GridColor = [0 0 0];
    ax.GridAlpha = 0.6;
    
    % Draw grid based on the computed gap:
    xticks(grid_xticks);
    yticks(grid_yticks);
end

%% Check for redundant values shen computing robot position
function[return_val] =  redundant_check(robot_u_xy, robot_s_pos, new_x, new_y)
    return_val = 1;
    for(i=1:size(robot_u_xy,1))
        compare_x = round(robot_u_xy(i,1),1);
        compare_y = round(robot_u_xy(i,2),1);
        if (round(new_x,1) == compare_x && round(new_y,1) == compare_y)
            return_val = 0;
        end
    end
    for(i=1:size(robot_s_pos,1))
        compare_x = round(robot_s_pos(i,1),1);
        compare_y = round(robot_s_pos(i,2),1);
        if (round(new_x,1) == compare_x && round(new_y,1) == compare_y)
            return_val = 0;
        end
    end
end
