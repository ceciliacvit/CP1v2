cellsize = 0.1;
map = createMap(cellsize);

start = [0.0 0.0];
goal = [1.5 0.5];

path = createPath(start, goal, map)