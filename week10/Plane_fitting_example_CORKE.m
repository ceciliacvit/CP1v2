%% CORKE Plane fitting - sec. 14.3.8
clear all
clc
close all
%% Plane fitting
% Apply an initial transform
T = trvec2tform([1,2,3])*eul2tform([pi/2 pi/8 pi/2]); 

% Create a grid of points
P = mkgrid(10, 1, 'pose', T); 

% Add noise to the points
P = P + 0.02*randn(size(P));

% Plot the points
scatter3(P(:,1), P(:,2), P(:,3))

% Calculate centroid (mean)
x0 = mean(P)'

% Calculate point coordinates with respect to centroid
% The points are column vectors
P0 = P' - mean(P)'

% Calculate the inertial matrix
J = P0*P0'  

% Get the eigen values
[x,lambda] = eig(J);

% smallest eigenvalue (from 3 in diag(lambda)
% corresponds to the eigenvector normal to the plane (in this case n)
diag(lambda)'
n = x(:,1)'
n2 = x(:,2)'
n3 = x(:,3)'

% Plot the vectors, first one is the normal to the plane
% The other two are plotted for completeness.
hold on
nvec = [x0' ; x0' + 0.5*n]; % normal vector for plotting (from mean to mean+normvec)
line(nvec(:,1), nvec(:,2), nvec(:,3), 'Color', 'r')
hold on
nvec = [x0' ; x0' + 0.5*n2]; % normal vector for plotting (from mean to mean+normvec)
line(nvec(:,1), nvec(:,2), nvec(:,3), 'Color', 'b')
hold on
nvec = [x0' ; x0' + 0.5*n3]; % normal vector for plotting (from mean to mean+normvec)
line(nvec(:,1), nvec(:,2), nvec(:,3), 'Color', 'g')