%map = createMap(0.1);
%figure
%show(map)

function map = createMap(cellsize)
% Create a simple occupancy map based on the cell size

% Map size in meters
map_width_m = 5;
map_height_m = 5;
robot_size_side = 0.07; % meters

% Resolution in cells per meter
res = 1 / cellsize;

% Map size in grid cells
map_width = round(map_width_m / cellsize);
map_height = round(map_height_m / cellsize);

% Create occupancy grid
simplegrid = binaryOccupancyMap(map_width, map_height, 'grid', res=res);

% Add one obstacle block
%i = 10;
%j = 10;
%simplegrid.setOccupancy([i j], ones(3,33), 'grid');

i = 20;
j = 10;
simplegrid.setOccupancy([i j], ones(33,3), 'grid')

% Inflate obstacles to account for robot size
simplegrid.inflate(robot_size_side);

% Return generated map
map = simplegrid;
end