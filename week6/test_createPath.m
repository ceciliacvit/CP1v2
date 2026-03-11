cellsize = 0.1;
map = createMap(cellsize);

start = [0.0 0.0];
goal = [2.5 1.5];

path = createPath(start, goal, map)