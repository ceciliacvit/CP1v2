 %% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Create map
simplegrid = binaryOccupancyMap(false(50,50));
simplegrid.setOccupancy([35 30],true(2,3),"grid");
simplegrid.setOccupancy([5 4],true(1,2),"grid");
simplegrid.setOccupancy([50-2 10], true(2,2), "grid");

%% Create planner
dx = DistanceTransformPlanner(simplegrid)

%% Create plan for reaching goal
dx.plan([2 2])

%% Euclidean or Manhattan 
dx = DistanceTransformPlanner(simplegrid,metric="manhattan");

%% Plan for goal
dx.plan([2 2]);

%% Get path from starting point
pathPoints = dx.query([35 23])

%% Plot path
figure
dx.plot(pathPoints)

%% Robot scale
% assumption robot occupies one cell. Some scale relating 1 cell to
% real-world sizefigure
figure
dx = DistanceTransformPlanner(simplegrid, inflate=2);
dx.plan([2 2]);
pathPoints = dx.query([35 23]);
dx.plot(pathPoints)
