function [obj_boundary, centroid_send] = ObjectDefine (I)
% Defines objects by reading the image file
% The function returns following objects:
% 1. Object boundary
% 2. Centroid of the object
% 

%% Reads the image:

BW = im2bw(I);

%% Compute the object boundary:
dim = size(BW);
col = round(dim(2)/2) - 90;
row = min(find(BW(:,col)));
obj_boundary = bwtraceboundary(BW,[row, col],'N'); % Return value

%% Get the geometry of the object (centroids)
[ geom, iner, cpmo ] = polygeom(obj_boundary(:,1), obj_boundary(:,2));
cent_x = geom(2); cent_y = geom(3);

%% Final reurn:
centroid_send = [cent_x, cent_y];

end
