cellsize = 0.1;
map = createMap(cellsize);

start = [0.1 0.1];
goal = [1.5 1.5];

path = createPath(start, goal, map)