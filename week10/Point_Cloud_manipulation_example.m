%% Point Cloud manipulation example
clear

load('PointCloud2Data.mat')

scatter3(pcd)

% read RGB
xrgb=reshape(pcd.readRGB(),640,480,3);
xrgb=permute(xrgb, [2 1 3]); % re-arrange from BGR to RGB
figure
imshow(xrgb)

% read XYZ format
xyz = pcd.readXYZ();
figure
scatter3(xyz(:,1),xyz(:,2), xyz(:,3))

xyzc = xyz(~isnan(xyz(:,3)),:); % remove NaN

%% kmeans for clustering

K = 20;
[idx,C,sumd] = kmeans(xyzc, K); 
%C = C'

figure
scatter3(xyz(:,1),xyz(:,2), xyz(:,3), '.')
hold on
for i=1:K,
    plot3(C(i,1), C(i,2), C(i,3), 'r.')
end

figure
for i=1:K
    scatter3(xyzc(idx==i,1),xyzc(idx==i,2), xyzc(idx==i,3), '.')
    hold on
end


%% Plane fitting on single cluster

M = 3; % cluster id
figure
scatter3(xyzc(idx==M,1),xyzc(idx==M,2), xyzc(idx==M,3), '.')

P = xyzc(idx==M,:)';    % Plane fitting..
N = size(P,2); % number points

x0 = mean(P')

figure, scatter3(pcd)

% SIMILAR TO Plane_fitting_example_CORKE.m
% Do it yourself :).


%% The alternative..

help pcfitplane

%% Or RANSAC
% see Corke 3rd edition pg. 640 for an example