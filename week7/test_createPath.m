cellsize = 0.1;
map = createMap(cellsize);

start = [0.0 0.0];
goal = [1.8 1.3];

path = createPath(start, goal, map)