function Drawgrid(obj_boundary, centroid, gap)
% Draws object and grid lines based on g_x and g_y
% The function has no returns
% 

%% Object plot:
figure(1)
xlimit = 500;
ylimit = 500;
xlim([0 xlimit]); ylim([100 ylimit]);
hold on;
fill(obj_boundary(:,1), obj_boundary(:,2), [0.8 0.8 0.8]);
plot(obj_boundary(:,1), obj_boundary(:,2), '-k', 'LineWidth',2);
alpha(.5)
plot(centroid(1), centroid(2),'rx','Linewidth',2, 'markersize',13);

%% Grid plot:
grid on;
% Grid properties:
ax = gca;
ax.GridColor = [0 0 0];
ax.GridAlpha = 0.8;

% Draw grid based on the computed gap:
temp_div = centroid(1)/gap(1);
start_x = centroid(1) - round(temp_div)*gap(1);
temp_div = centroid(2)/gap(2);
start_y = centroid(2) - round(temp_div)*gap(2);
xticks(round(start_x,2):round(gap(1),2):xlimit);
yticks(round(start_y,2):round(gap(2),2):ylimit);
set(gca,'FontSize',15)
xlabel('x^g') 
ylabel('y^g') 
