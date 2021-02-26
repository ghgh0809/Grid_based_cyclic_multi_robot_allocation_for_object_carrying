clear; clc; close all;

v = VideoWriter('sample.avi');
v.FrameRate = 60;
v.Quality = 100;

no_edge = 6;  % Define number of edge of polygon first
y_gap = 0.05;  % break down gap of polygon (y-axis)
x_gap = y_gap;
A_tolerance = 0; % Due to momentum, there should be tolerance area to be safe from unstability
n=0;
circ_n = 100;
r_robot_1 = 1; % Radius of spherical robot
r_robot_2 = 1.3;

%% Define location of the edge of polygon
Fig = figure(1);
open(v);
set(Fig, 'Position', [100 100 800 600])
set(gcf,'color','w');

% 
% subplot(2,2,1);
% for i =1:no_edge
%     title(['Defining shape of the object: ' num2str(no_edge) ' edges']);
%     n(n+1) = n+1;
%     xlim([0 30]); ylim([0 30]);
%     [edge_loc_x, edge_loc_y] = ginput(1);
%     plot(edge_loc_x, edge_loc_y,'ko');
%     xlim([0 30]); ylim([0 30]);
%     s_edge_loc_x(n(end)) = edge_loc_x;
%     s_edge_loc_y(n(end)) = edge_loc_y;
%     hold on; grid off;
% end
% n(n+1) = n+1;
% s_edge_loc_x(n(end)) = s_edge_loc_x(1);
% s_edge_loc_y(n(end)) = s_edge_loc_y(1);

subplot(2,2,1);
set(gca,'FontSize',15)
xlim([0 30]); ylim([0 30]);
xlabel('x^{g}')
ylabel('y^{g}')
hold on; grid off;

I = imread('obj2.png');
BW = im2bw(I);
dim = size(BW);

col = round(dim(2)/2)-90;
row = min(find(BW(:,col)));

obj_boundary = bwtraceboundary(BW,[row, col],'N');

max_b = max(obj_boundary);
min_b = min(obj_boundary);
size_b = size(obj_boundary);

a = transpose(obj_boundary(:,2)/max_b(2)*25);
b = transpose(obj_boundary(:,1)/max_b(1)*25);
s_edge_loc_x = a;
s_edge_loc_y = b;

%% Draw the polygon
patch(s_edge_loc_x, s_edge_loc_y, [0.85,0.85,0.85], 'EdgeColor','black','LineWidth',2);

%% Find the centroid of the polygon
% polyin = polyshape(s_edge_loc_x, s_edge_loc_y);
% [cent_x, cent_y] = centroid(polyin);

measurements = regionprops(BW, 'Centroid');
centroids = cat(1,measurements.Centroid);
cent_x = centroids(:,1)/max_b(2)*25;
cent_y = centroids(:,2)/max_b(1)*25;

plot(cent_x, cent_y,'rx','Linewidth',2, 'markersize',13);
hold on;

%% Find edge points of the polygon
% n = 0;
% s_x_pol = 0; s_y_pol = 0;
% for i=1:1:no_edge
%     n(n+1) = n+1;
%     x_pol = linspace(s_edge_loc_x(i), s_edge_loc_x(i+1), 200);
%     s_x_pol = [s_x_pol x_pol];
% end
% for i=1:1:no_edge
%     n(n+1) = n+1;
%     y_pol = linspace(s_edge_loc_y(i),s_edge_loc_y(i+1), 200);
%     s_y_pol = [s_y_pol y_pol];
% end
% for i=1:1:no_edge
%     s_x_pol(i) = s_x_pol(i+1);
%     s_y_pol(i) = s_y_pol(i+1);
% end
% plot(s_x_pol, s_y_pol, 'ro');

s_x_pol = s_edge_loc_x; s_y_pol = s_edge_loc_y;

%% Find vertical gap
n = 0;
a_size = size(s_x_pol);
for i = 1:1:a_size(1,2)
    if round(s_y_pol(i),1) == round(cent_y,1)
        n(n+1) = n+1;
        gap_v_option(n(end)) = abs(s_x_pol(i) - cent_x);
    end
end
gap_v = min(gap_v_option) - A_tolerance;

%% Find the point where the horizontal gap of the polygon is equal to vertical gap
m = 0;
hor_gap=0;
size_obj = size(s_y_pol);
for i = 1:1:size_obj(1,2)
    comp_val_1 = round(s_y_pol(i),1);
    n = 1;
    s_x_gap_point = 0;
    for j = i+1:1:size_obj(1,2)
        comp_val_2 = round(s_y_pol(j),1);
        if comp_val_1 == comp_val_2
            s_x_gap_point(n(end)) = s_x_pol(i);
            n(n+1) = n+1;
            s_x_gap_point(n(end)) = s_x_pol(j);
            gap_comp = max(s_x_gap_point) - min(s_x_gap_point);
            if round(gap_comp,1) == round(gap_v,1)
                m(m+1) = m+1;
                gap_h_option(m(end)) = abs(s_y_pol(j)-cent_y);
            end
        end
    end
end
gap_y = min(gap_h_option) - A_tolerance;
%% Draw grid with defined horizontal gap
for i=0:50
    line([cent_x-(gap_v)*i, cent_x-(gap_v)*i], get(gca, 'ylim'),'Color',[0 0 0],'LineStyle',':');
    line([cent_x+(gap_v)*i, cent_x+(gap_v)*i], get(gca, 'ylim'),'Color',[0 0 0],'LineStyle',':');
    line(get(gca, 'xlim'),[cent_y-(gap_y)*i, cent_y-(gap_y)*i],'Color',[0 0 0],'LineStyle',':');
    line(get(gca, 'xlim'),[cent_y+(gap_y)*i, cent_y+(gap_y)*i],'Color',[0 0 0],'LineStyle',':');
end



%% Open new figure for the simulation and plot object's shape and computed grid

figure(1)
pos = [0.13 0.1 0.79 0.35];
subplot('Position', pos);
grid off;
xlim([0 100]); ylim([-5 35]);
for i=0:50
    line([cent_x-(gap_v-A_tolerance)*i, cent_x-(gap_v-A_tolerance)*i], get(gca, 'ylim'),'Color',[0 0 0],'LineStyle',':');
    line([cent_x+(gap_v-A_tolerance)*i, cent_x+(gap_v-A_tolerance)*i], get(gca, 'ylim'),'Color',[0 0 0],'LineStyle',':');
    line(get(gca, 'xlim'),[cent_y-(gap_y-A_tolerance)*i, cent_y-(gap_y-A_tolerance)*i],'Color',[0 0 0],'LineStyle',':');
    line(get(gca, 'xlim'),[cent_y+(gap_y-A_tolerance)*i, cent_y+(gap_y-A_tolerance)*i],'Color',[0 0 0],'LineStyle',':');
end

%% Apply animation of figure 2 to simulate movement of the object
tic;
n = 0;
a = 0;
s_contact_p_x=0;
s_contact_p_y=0;
iteration = 0;
while (toc < 150)
    iteration = iteration+1;
    frame = getframe(gcf);
    writeVideo(v, frame);
    subplot('Position', pos);
    title(['Movement of the object [i :' num2str(iteration) ']']);
    cla;
    s_x_pol_new = s_x_pol + toc/5;
    s_edge_loc_x_new = s_edge_loc_x + toc/5;
    patch(s_edge_loc_x_new, s_edge_loc_y, [0.85,0.85,0.85], 'EdgeColor','black','LineWidth',1.2);
    cent_x_new = cent_x + toc/5;
    hold on;
    set(gca,'FontSize',15)
    plot(cent_x_new, cent_y,'rx','Linewidth',2, 'markersize',13);
    % plot(s_x_pol_new, s_y_pol,'ro');
    % Drawing grid lines
    for i=0:50
        line([cent_x-(gap_v-A_tolerance)*i, cent_x-(gap_v-A_tolerance)*i], get(gca, 'ylim'),'Color',[0.70,0.70,0.70],'LineStyle','-','LineWidth',1.2);
        line([cent_x+(gap_v-A_tolerance)*i, cent_x+(gap_v-A_tolerance)*i], get(gca, 'ylim'),'Color',[0.70,0.70,0.70],'LineStyle','-','LineWidth',1.2);
        line(get(gca, 'xlim'),[cent_y-(gap_y-A_tolerance)*i, cent_y-(gap_y-A_tolerance)*i],'Color',[0.70,0.70,0.70],'LineStyle','-','LineWidth',1.2);
        line(get(gca, 'xlim'),[cent_y+(gap_y-A_tolerance)*i, cent_y+(gap_y-A_tolerance)*i],'Color',[0.70,0.70,0.70],'LineStyle','-','LineWidth',1.2);
    end
    n=0;
    % Get grid positions
    for i=0:100
        if  (cent_x-(gap_v-A_tolerance)*i >= 0) && (cent_x+(gap_v-A_tolerance)*(i) <= 100)
            n(n+1) = n+1;
            s_x_grid(n(end)) = cent_x-(gap_v-A_tolerance)*i;
            n(n+1) = n+1;
            s_x_grid(n(end)) = cent_x+(gap_v-A_tolerance)*i;
        end
        if  (cent_x-(gap_v-A_tolerance)*i < 0) && (cent_x+(gap_v-A_tolerance)*(i) <= 100)
            n(n+1) = n+1;
            s_x_grid(n(end)) = cent_x+(gap_v-A_tolerance)*i;
        end
    end
    n=0;
    for i=0:100
        if  (cent_y-(gap_y-A_tolerance)*i >= 0) && (cent_y+(gap_y-A_tolerance)*(i) <= 100)
            n(n+1) = n+1;
            s_y_grid(n(end)) = cent_y-(gap_y-A_tolerance)*i;
            n(n+1) = n+1;
            s_y_grid(n(end)) = cent_y+(gap_y-A_tolerance)*i;
        end
        if  (cent_y-(gap_y-A_tolerance)*i < 0) && (cent_y+(gap_y-A_tolerance)*(i) <= 100)
            n(n+1) = n+1;
            s_y_grid(n(end)) = cent_y+(gap_y-A_tolerance)*i;
        end
    end
    s_x_grid = sort(s_x_grid);
    s_y_grid = sort(s_y_grid);
    size_s_x_grid = size(s_x_grid);
    size_s_y_grid = size(s_y_grid);
    ver_poly_x = s_edge_loc_x_new;
    ver_poly_y = s_edge_loc_y;
    ver_poly_x(no_edge+1) = ver_poly_x(1);
    ver_poly_y(no_edge+1) = ver_poly_y(1);
    n = 0;
    for i=1:1:size_s_x_grid(2)
        for j=1:1:size_s_y_grid(2)
            n(n+1) = n+1;
            s_x_total_grid(n(end)) = s_x_grid(i);
            s_y_total_grid(n(end)) = s_y_grid(j);
        end
    end
    [in, on] = inpolygon(s_x_total_grid,s_y_total_grid,ver_poly_x,ver_poly_y);
    s_x_inner_pol_edge = s_x_total_grid(in);
    s_y_inner_pol_edge = s_y_total_grid(in);
    s_x_inner_pol_edge = reshape(s_x_inner_pol_edge,[],1);
    s_y_inner_pol_edge = reshape(s_y_inner_pol_edge,[],1);
    
    inner_pol_edge_temp = [];
    
    inner_pol_edge_temp(:,1) = s_x_inner_pol_edge;
    inner_pol_edge_temp(:,2) = s_y_inner_pol_edge;
    inner_pol_edge_temp = unique(inner_pol_edge_temp,'rows');
    
    s_x_inner_pol_edge = inner_pol_edge_temp(:,1);
    s_y_inner_pol_edge = inner_pol_edge_temp(:,2);
    
    s_x_inner_pol_edge = [s_x_inner_pol_edge(end); s_x_inner_pol_edge];
    s_y_inner_pol_edge = [s_y_inner_pol_edge(end); s_y_inner_pol_edge];
    
    j = boundary(s_x_inner_pol_edge, s_y_inner_pol_edge, 0.1);
    s_x_boun_in_pol_edge = s_x_inner_pol_edge(j);
    s_y_boun_in_pol_edge = s_y_inner_pol_edge(j);

%     xlabel('traveled distance')
%     ylabel('y')
    
    yellow_v = [0.9290 0.6940 0.1250];
    blue_v = [0 0.4470 0.7410];
    black_v = [0 0 0];
    green_v = [0.4660 0.6740 0.1880];

    plot(s_x_total_grid(in)+(gap_v-A_tolerance),s_y_total_grid(in)+(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in)-(gap_v-A_tolerance),s_y_total_grid(in)-(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in)+(gap_v-A_tolerance),s_y_total_grid(in)-(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in)-(gap_v-A_tolerance),s_y_total_grid(in)+(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in)+(gap_v-A_tolerance),s_y_total_grid(in),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in)-(gap_v-A_tolerance),s_y_total_grid(in),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in),s_y_total_grid(in)+(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in),s_y_total_grid(in)-(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1., 'MarkerFaceColor', blue_v);
    
    
    plot(s_x_total_grid(on)+(gap_v-A_tolerance),s_y_total_grid(on)+(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(on)-(gap_v-A_tolerance),s_y_total_grid(on)-(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(on)+(gap_v-A_tolerance),s_y_total_grid(on)-(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(on)-(gap_v-A_tolerance),s_y_total_grid(on)+(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in)+(gap_v-A_tolerance),s_y_total_grid(in),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in)-(gap_v-A_tolerance),s_y_total_grid(in),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in),s_y_total_grid(in)-(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    plot(s_x_total_grid(in),s_y_total_grid(in)+(gap_y-A_tolerance),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', blue_v);
    
    plot(s_x_total_grid(in),s_y_total_grid(in),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', green_v);
    plot(s_x_total_grid(on),s_y_total_grid(on),'o', 'MarkerEdgeColor',black_v, 'markersize', 10, 'Linewidth',1.5, 'MarkerFaceColor', green_v);

    s = fill(s_x_boun_in_pol_edge,s_y_boun_in_pol_edge,green_v);
    alpha(s, 0.5);
    
    %% Find edge points of the inner polygon
    n = 0;
    f = 0;
    s_x_inner_pol = 0; s_y_inner_pol = 0;
    for i=1:1:size(s_x_boun_in_pol_edge,1)-1
        n(n+1) = n+1;
        x_pol = linspace(s_x_boun_in_pol_edge(i), s_x_boun_in_pol_edge(i+1), 500);
        s_x_inner_pol = [s_x_inner_pol x_pol];
    end
    for i=1:1:size(s_y_boun_in_pol_edge,1)-1
        n(n+1) = n+1;
        y_pol = linspace(s_y_boun_in_pol_edge(i), s_y_boun_in_pol_edge(i+1), 500);
        s_y_inner_pol = [s_y_inner_pol y_pol];
    end
    for i=1:1:size(s_y_boun_in_pol_edge,1)-1
        s_x_inner_pol(i) = s_x_inner_pol(i+1);
        s_y_inner_pol(i) = s_y_inner_pol(i+1);
    end
    % plot(s_x_inner_pol, s_y_inner_pol, 'ro');
    for i=1:1:size(s_y_inner_pol,2)
        f(f+1) = f+1;
        dis_point = [cent_x_new, cent_y; s_x_inner_pol(i), s_y_inner_pol(i)];
        dist(f(end)) = pdist(dis_point,'euclidean');
    end
    a(a+1) = a+1;
    min_dist(a(end)) = min(dist);
    min_dist_x(a(end)) = iteration;
    subplot(2,2,2);
    %title(['Shortest distance from centroid and inner polygon']);
    set(gca,'FontSize',15)
    xlabel('iteration')
    ylabel('s_{c,R}')
    plot(min_dist_x, min_dist, 'r-');
    hold on;
    xlim([0 300])
    ylim([0 7])
    %pause(0.01)
    
end

close(v);
%% Open new figure to show shape of figure with gaps
figure(2);
set(figure(2), 'Position', [100 100 700 600])
hold on;
% title(['Object with generated grid']);
x_ticks = [];
y_ticks = [];
tickxmin_g = 1; tickxmax_g = 1; tickymin_g = 1; tickymax_g = 1;
while (0 < (cent_x-(gap_v)*tickxmin_g))
    tickxmin_g =  tickxmin_g + 1;
end
while (30 > (cent_x+(gap_v)*tickxmax_g))
    tickxmax_g = tickxmax_g + 1;
end
while (0 < (cent_y-(gap_y)*tickymin_g))
    tickymin_g =  tickymin_g + 1;
end
while (30 > (cent_x+(gap_v)*tickymax_g))
    tickymax_g = tickymax_g + 1;
end
xlim([0 30])
ylim([0 30])
% xlim([round(cent_x-(gap_v)*tickxmin_g,1) round(cent_x+(gap_v)*tickxmax_g,1)]);
% ylim([round(cent_x-(gap_y)*tickymin_g,1) round(cent_x+(gap_y)*tickymax_g,1)]);

while (tickxmin_g > 0)
    x_ticks = [x_ticks, round(cent_x-(gap_v)*tickxmin_g,1)];
    tickxmin_g = tickxmin_g - 1;
end
while (tickxmax_g >= 0)
    x_ticks = [x_ticks, round(cent_x+(gap_v)*tickxmax_g,1)];
    tickxmax_g = tickxmax_g - 1;
end
x_ticks = sort(x_ticks);

while (tickymin_g > 0)
    y_ticks = [y_ticks, round(cent_y-(gap_y)*tickymin_g,1)];
    tickymin_g = tickymin_g - 1;
end
while (tickymax_g >= 0)
    y_ticks = [y_ticks, round(cent_y+(gap_y)*tickymax_g,1)];
    tickymax_g = tickymax_g - 1;
end
y_ticks = sort(y_ticks);

patch(s_edge_loc_x, s_edge_loc_y, [0.85,0.85,0.85], 'EdgeColor','black','LineWidth',3.5);
plot(cent_x, cent_y,'rx','Linewidth',1.2, 'markersize',13);
set(gca,'FontSize',25)
% plot(cent_x, cent_y, 'color',[1 1 1], marker,'x', linewidth,2, markersize,12);
xlabel('x^{g}')
ylabel('y^{g}')
xticks(x_ticks);
yticks(y_ticks);
for i=0:100
    line([cent_x-(gap_v)*i, cent_x-(gap_v)*i], get(gca, 'ylim'),'Color',[0.7 0.7 0.7],'LineStyle','-','LineWidth',2);
    line([cent_x+(gap_v)*i, cent_x+(gap_v)*i], get(gca, 'ylim'),'Color',[0.7 0.7 0.7],'LineStyle','-','LineWidth',2);
    line(get(gca, 'xlim'),[cent_y-(gap_y)*i, cent_y-(gap_y)*i],'Color',[0.7 0.7 0.7],'LineStyle','-','LineWidth',2);
    line(get(gca, 'xlim'),[cent_y+(gap_y)*i, cent_y+(gap_y)*i],'Color',[0.7 0.7 0.7],'LineStyle','-','LineWidth',2);
end

figure(3)
set(figure(3), 'Position', [100 100 700 600])
hold on;
plot(min_dist_x, min_dist, 'r-','LineWidth',2);
xlim([0 200]); ylim([0 7]);
% title(['Shortest distance from centroid and inner polygon']);
set(gca,'FontSize',23)
xlabel('iteration')
ylabel('s_{c,R}')
xticks([0 40 80 120 160 200]);
yticks([0 1.8 3.6 5.4 7.2 9]);
grid on;
hold on;
