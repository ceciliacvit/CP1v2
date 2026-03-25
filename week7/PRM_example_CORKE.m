% Examples of PRM methods
clear all
clc
close all

%% Initialization - create map, set start and goal points
% 1.1m x 1.2m
% cellsize = 10cm x 10cm
% robot size -> 15cm x 15cm (xy base, only approx)

% Determine map scale
cellsize = 0.1; % in meters
map_width = round(1.2 / cellsize); % in meters
map_height = round(1.1 / cellsize); % in meters
robot_size_side = 0.07; % in meters

% Create occupancy grid and add obstacles
simplegrid = binaryOccupancyMap(map_width, map_height, 'grid', res=10);
i = [6];
j = [6];
simplegrid.setOccupancy([i j],ones(7,7),'grid');

% Inflate obstacles
simplegrid.inflate(robot_size_side)
%% Set start and goal locations
start = [0.1 0.1];
goal = [1.0 1.0];

%% Probabilistic Roadmap method
figure
prm = mobileRobotPRM(simplegrid); % create prm planner
prm.show() % plot the roadmap

path = prm.findpath(start, goal) % query planner for path
prm.show() % show path
hold on, plot(start(1), start(2), 'r*', 'MarkerSize', 20), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro', 'MarkerSize', 20), text(goal(1), goal(2), 'GOAL')

%% Experiment with the number of nodes
%%Prøv med lidt forskellige nodes
%%prm.NumNodes = 5;
prm.NumNodes = 25;
%%prm.NumNodes = 100;
prm.show()
